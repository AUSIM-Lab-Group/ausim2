#include "gui_control_model.hpp"
#include "gui_runtime_config.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <QApplication>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFocusEvent>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLayout>
#include <QLineEdit>
#include <QListWidget>
#include <QPaintEvent>
#include <QPainter>
#include <QPushButton>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace remote_control {
namespace {

constexpr char kDefaultJoyTopic[] = "/joy";
constexpr char kDefaultCmdVelTopic[] = "/joy/cmd_vel";

double ApplyDeadzone(double value, double deadzone) {
  if (std::abs(value) < deadzone) {
    return 0.0;
  }
  return value;
}

char NormalizeKey(char key) { return static_cast<char>(std::tolower(static_cast<unsigned char>(key))); }

std::optional<char> QtKeyToChar(int key) {
  if (key >= Qt::Key_A && key <= Qt::Key_Z) {
    return static_cast<char>('a' + (key - Qt::Key_A));
  }
  if (key >= Qt::Key_0 && key <= Qt::Key_9) {
    return static_cast<char>('0' + (key - Qt::Key_0));
  }
  if (key == Qt::Key_Space) {
    return ' ';
  }
  return std::nullopt;
}

std::string CurrentTimeText() { return QDateTime::currentDateTime().toString("HH:mm:ss").toStdString(); }

QString ButtonsText(const std::vector<int>& buttons) {
  QStringList parts;
  for (const int button : buttons) {
    if (button >= 0) {
      parts << QString::number(button);
    }
  }
  return parts.join(",");
}

std::vector<int> ParseButtonsText(QString text) {
  text = text.trimmed();
  if (text.isEmpty() || text == "-1" || text.compare("none", Qt::CaseInsensitive) == 0) {
    return {};
  }

  text.replace(';', ',');
  text.replace(' ', ',');
  const QStringList tokens = text.split(',', Qt::SkipEmptyParts);
  std::vector<int> buttons;
  for (const QString& token : tokens) {
    bool ok = false;
    const int button = token.trimmed().toInt(&ok);
    if (!ok) {
      throw std::runtime_error("invalid joystick button index: " + token.toStdString());
    }
    if (button >= 0) {
      buttons.push_back(button);
    }
  }
  return buttons;
}

class AxisBarWidget final : public QWidget {
 public:
  explicit AxisBarWidget(QWidget* parent = nullptr) : QWidget(parent) {
    setMinimumSize(24, 62);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }

  void SetValue(double value) {
    value_ = std::clamp(value, -1.0, 1.0);
    update();
  }

 protected:
  void paintEvent(QPaintEvent* event) override {
    (void)event;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRect bar = rect().adjusted(4, 3, -4, -3);
    painter.setPen(QPen(QColor(120, 120, 120)));
    painter.setBrush(QColor(245, 245, 245));
    painter.drawRoundedRect(bar, 3, 3);

    const int center_y = bar.top() + bar.height() / 2;
    const int fill_height = static_cast<int>(std::round(std::abs(value_) * (bar.height() / 2.0)));
    if (fill_height > 0) {
      const QRect fill =
          value_ >= 0.0 ? QRect(bar.left(), center_y - fill_height, bar.width(), fill_height) : QRect(bar.left(), center_y, bar.width(), fill_height);
      painter.setPen(Qt::NoPen);
      painter.setBrush(value_ >= 0.0 ? QColor(52, 152, 219) : QColor(230, 126, 34));
      painter.drawRoundedRect(fill, 3, 3);
    }

    painter.setPen(QPen(QColor(40, 40, 40), 2));
    painter.drawLine(bar.left() - 2, center_y, bar.right() + 2, center_y);
  }

 private:
  double value_ = 0.0;
};

class ButtonDotWidget final : public QWidget {
 public:
  explicit ButtonDotWidget(QWidget* parent = nullptr) : QWidget(parent) { setFixedSize(20, 20); }

  void SetPressed(bool pressed) {
    pressed_ = pressed;
    update();
  }

 protected:
  void paintEvent(QPaintEvent* event) override {
    (void)event;
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF circle = rect().adjusted(2, 2, -2, -2);
    painter.setPen(QPen(pressed_ ? QColor(20, 120, 45) : QColor(150, 30, 30), 1));
    painter.setBrush(pressed_ ? QColor(35, 180, 80) : QColor(230, 70, 70));
    painter.drawEllipse(circle);
  }

 private:
  bool pressed_ = false;
};

}  // namespace

struct GuiActionBinding {
  std::string service_name;
  std::vector<int> buttons;
  char keyboard_key = '\0';
};

struct ActionRowWidgets {
  QLabel* name_label = nullptr;
  QLineEdit* service_edit = nullptr;
  QLineEdit* buttons_edit = nullptr;
  QLineEdit* keyboard_edit = nullptr;
  QPushButton* capture_button = nullptr;
  QPushButton* clear_button = nullptr;
};

struct AxisStateWidgets {
  QWidget* container = nullptr;
  QLabel* name_label = nullptr;
  AxisBarWidget* bar = nullptr;
  QLabel* value_label = nullptr;
};

struct ButtonStateWidgets {
  QWidget* container = nullptr;
  ButtonDotWidget* dot = nullptr;
  QLabel* name_label = nullptr;
};

struct AxisConfig {
  int linear_x = 4;
  int linear_y = 0;
  int linear_z = 1;
  int angular_yaw = 3;
};

struct GuiConfig {
  std::string joy_topic = kDefaultJoyTopic;
  std::string cmd_vel_topic = kDefaultCmdVelTopic;
  double publish_rate_hz = 30.0;
  double joy_timeout_seconds = 0.5;
  double deadzone = 0.15;
  bool require_enable_button = false;
  int enable_button = 4;
  AxisConfig axes;
  MotionScale joystick_scale;
  MotionScale keyboard_scale;
  bool keyboard_enabled = true;
  double command_cooldown_seconds = 0.5;
  std::chrono::steady_clock::duration motion_suppress_duration{std::chrono::milliseconds(300)};
  std::unordered_map<std::string, GuiActionBinding> actions;
  std::string config_path;
  std::string language = "zh";
};

class RemoteControlGuiWindow final : public QWidget {
 public:
  explicit RemoteControlGuiWindow(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)), model_(30) {
    LoadParameters();
    BuildUi();
    SetupRosInterfaces();

    const int interval_ms = std::max(10, static_cast<int>(1000.0 / config_.publish_rate_hz));
    publish_timer_ = new QTimer(this);
    connect(publish_timer_, &QTimer::timeout, this, [this]() { OnTimer(); });
    publish_timer_->start(interval_ms);

    setFocusPolicy(Qt::StrongFocus);
    setFocus();
    RefreshUi("Idle");
  }

 protected:
  void keyPressEvent(QKeyEvent* event) override {
    const std::optional<char> key = QtKeyToChar(event->key());
    if (!key.has_value()) {
      QWidget::keyPressEvent(event);
      return;
    }

    const char normalized = NormalizeKey(*key);
    if (normalized == ' ') {
      model_.ClearKeyboard();
      PublishZeroCommand();
      RefreshUi("Keyboard");
      event->accept();
      return;
    }

    if (IsMovementKey(normalized) && config_.keyboard_enabled) {
      model_.PressKey(normalized);
      RefreshUi("Keyboard");
    }

    if (!event->isAutoRepeat()) {
      const std::optional<std::string> action_name = KeyboardActionForKey(normalized);
      if (action_name.has_value()) {
        TriggerAction(*action_name, "keyboard");
      }
    }

    event->accept();
  }

  void keyReleaseEvent(QKeyEvent* event) override {
    if (event->isAutoRepeat()) {
      event->accept();
      return;
    }

    const std::optional<char> key = QtKeyToChar(event->key());
    if (!key.has_value()) {
      QWidget::keyReleaseEvent(event);
      return;
    }

    const char normalized = NormalizeKey(*key);
    if (IsMovementKey(normalized)) {
      model_.ReleaseKey(normalized);
      RefreshUi("Keyboard");
      event->accept();
      return;
    }

    QWidget::keyReleaseEvent(event);
  }

  void focusOutEvent(QFocusEvent* event) override {
    model_.ClearKeyboard();
    PublishZeroCommand();
    RefreshUi("Idle");
    QWidget::focusOutEvent(event);
  }

  void resizeEvent(QResizeEvent* event) override {
    QWidget::resizeEvent(event);
    RelayoutJoyStateWidgets();
  }

 private:
  void LoadParameters() {
    config_.config_path = node_->declare_parameter<std::string>("gui.config_path", "");
    config_.joy_topic = node_->declare_parameter<std::string>("topics.joy", kDefaultJoyTopic);
    config_.cmd_vel_topic = node_->declare_parameter<std::string>("topics.cmd_vel", kDefaultCmdVelTopic);
    config_.publish_rate_hz = node_->declare_parameter<double>("publish_rate_hz", 30.0);
    config_.joy_timeout_seconds = node_->declare_parameter<double>("joy_timeout", 0.5);
    config_.deadzone = node_->declare_parameter<double>("deadzone", 0.15);
    config_.require_enable_button = node_->declare_parameter<bool>("require_enable_button", false);
    config_.enable_button = node_->declare_parameter<int>("enable_button", 4);
    config_.axes.linear_x = node_->declare_parameter<int>("axes.linear.x", 4);
    config_.axes.linear_y = node_->declare_parameter<int>("axes.linear.y", 0);
    config_.axes.linear_z = node_->declare_parameter<int>("axes.linear.z", 1);
    config_.axes.angular_yaw = node_->declare_parameter<int>("axes.angular.yaw", 3);
    config_.joystick_scale.linear_x = node_->declare_parameter<double>("scale.linear.x", 0.6);
    config_.joystick_scale.linear_y = node_->declare_parameter<double>("scale.linear.y", 0.6);
    config_.joystick_scale.linear_z = node_->declare_parameter<double>("scale.linear.z", 0.5);
    config_.joystick_scale.angular_yaw = node_->declare_parameter<double>("scale.angular.yaw", 1.0);
    config_.command_cooldown_seconds = node_->declare_parameter<double>("command_cooldown", 0.5);
    config_.keyboard_enabled = node_->declare_parameter<bool>("keyboard.enabled", true);
    (void)node_->declare_parameter<double>("keyboard.key_timeout", 0.5);
    config_.keyboard_scale.linear_x = node_->declare_parameter<double>("keyboard.scale.linear.x", 0.6);
    config_.keyboard_scale.linear_y = node_->declare_parameter<double>("keyboard.scale.linear.y", 0.6);
    config_.keyboard_scale.linear_z = node_->declare_parameter<double>("keyboard.scale.linear.z", 0.5);
    config_.keyboard_scale.angular_yaw = node_->declare_parameter<double>("keyboard.scale.angular.yaw", 1.0);
    config_.language = node_->declare_parameter<std::string>("gui.language", "zh") == "en" ? "en" : "zh";
    config_.motion_suppress_duration = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(node_->declare_parameter<double>("motion_suppress_after_command", 0.3)));

    if (config_.publish_rate_hz <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }
    if (config_.joy_timeout_seconds < 0.0) {
      throw std::runtime_error("joy_timeout must be non-negative");
    }

    LoadActionBindings();
    editable_settings_ = BuildEditableSettingsFromRuntime();
    if (!config_.config_path.empty()) {
      try {
        editable_settings_ = LoadEditableGuiSettingsFromYaml(config_.config_path);
        ApplyEditableSettingsToRuntime(editable_settings_);
      } catch (const std::exception& error) {
        RCLCPP_WARN(node_->get_logger(), "failed to load GUI editable settings from %s: %s", config_.config_path.c_str(), error.what());
      }
    }
    language_ = editable_settings_.language == "en" ? "en" : "zh";
  }

  void LoadActionBindings() {
    struct ActionDefault {
      const char* name;
      const char* service_name;
      std::vector<std::int64_t> buttons;
      char keyboard_key;
    };
    const ActionDefault defaults[] = {
        {"action1", "/joy/action1", {4, 0}, 't'},
        {"action2", "/joy/action2", {5, 0}, 'g'},
        {"action3", "/joy/action3", {6, 7}, 'x'},
        {"action4", "/joy/action4", {-1}, 'q'},
        {"action5", "", {-1}, '\0'},
        {"action6", "", {-1}, '\0'},
        {"action7", "", {-1}, '\0'},
        {"action8", "", {-1}, '\0'},
    };

    for (const ActionDefault& def : defaults) {
      const std::string name(def.name);
      const std::string service_name = node_->declare_parameter<std::string>("actions." + name + ".service", def.service_name);
      const std::vector<std::int64_t> raw_buttons = node_->declare_parameter<std::vector<std::int64_t>>("actions." + name + ".buttons", def.buttons);
      const std::string keyboard =
          node_->declare_parameter<std::string>("actions." + name + ".keyboard", def.keyboard_key == '\0' ? "" : std::string(1, def.keyboard_key));

      if (service_name.empty()) {
        continue;
      }

      GuiActionBinding binding;
      binding.service_name = service_name;
      binding.keyboard_key = keyboard.empty() ? '\0' : NormalizeKey(keyboard.front());
      for (const std::int64_t button : raw_buttons) {
        if (button >= 0) {
          binding.buttons.push_back(static_cast<int>(button));
        }
      }

      config_.actions.emplace(name, std::move(binding));
    }
  }

  EditableGuiSettings BuildEditableSettingsFromRuntime() const {
    EditableGuiSettings settings;
    settings.axis_mapping.linear_x = config_.axes.linear_x;
    settings.axis_mapping.linear_y = config_.axes.linear_y;
    settings.axis_mapping.linear_z = config_.axes.linear_z;
    settings.axis_mapping.angular_yaw = config_.axes.angular_yaw;
    settings.joystick_scale = config_.joystick_scale;
    settings.keyboard_scale = config_.keyboard_scale;
    settings.language = config_.language;
    settings.action_slots = DefaultEditableActionSlots();

    for (GuiActionSlotConfig& slot : settings.action_slots) {
      const auto binding_it = config_.actions.find(slot.name);
      if (binding_it == config_.actions.end()) {
        continue;
      }
      slot.service_name = binding_it->second.service_name;
      slot.buttons = binding_it->second.buttons;
      slot.keyboard_key = binding_it->second.keyboard_key == '\0' ? "" : std::string(1, binding_it->second.keyboard_key);
    }

    return settings;
  }

  void ApplyEditableSettingsToRuntime(const EditableGuiSettings& settings) {
    config_.axes.linear_x = settings.axis_mapping.linear_x;
    config_.axes.linear_y = settings.axis_mapping.linear_y;
    config_.axes.linear_z = settings.axis_mapping.linear_z;
    config_.axes.angular_yaw = settings.axis_mapping.angular_yaw;
    config_.joystick_scale = settings.joystick_scale;
    config_.keyboard_scale = settings.keyboard_scale;
    config_.language = settings.language == "en" ? "en" : "zh";
    config_.actions.clear();

    for (const GuiActionSlotConfig& slot : settings.action_slots) {
      if (slot.service_name.empty()) {
        continue;
      }

      GuiActionBinding binding;
      binding.service_name = slot.service_name;
      binding.buttons = slot.buttons;
      binding.keyboard_key = slot.keyboard_key.empty() ? '\0' : NormalizeKey(slot.keyboard_key.front());
      config_.actions.emplace(slot.name, std::move(binding));
    }
  }

  void BuildUi() {
    setWindowTitle("AUSIM Remote Control");
    resize(920, 760);
    setMinimumSize(920, 640);

    auto* root = new QVBoxLayout(this);
    auto* grid = new QGridLayout();

    mode_label_ = new QLabel(this);
    hint_label_ = new QLabel(this);
    hint_label_->setWordWrap(true);
    hint_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    history_list_ = new QListWidget(this);
    history_list_->setMaximumHeight(96);

    mode_title_label_ = new QLabel(this);
    grid->addWidget(mode_title_label_, 0, 0);
    grid->addWidget(mode_label_, 0, 1);

    root->addLayout(grid);
    root->addWidget(hint_label_);

    BuildJoystickStatePanel(root);
    BuildVelocityEditor(root);
    BuildActionEditor(root);

    auto* button_row = new QHBoxLayout();
    language_button_ = new QPushButton(this);
    save_button_ = new QPushButton(this);
    save_status_label_ = new QLabel(this);
    button_row->addWidget(language_button_);
    button_row->addWidget(save_button_);
    button_row->addWidget(save_status_label_, 1);
    root->addLayout(button_row);

    action_history_title_label_ = new QLabel(this);
    root->addWidget(action_history_title_label_);
    root->addWidget(history_list_, 0);

    connect(language_button_, &QPushButton::clicked, this, [this]() { ToggleLanguage(); });
    connect(save_button_, &QPushButton::clicked, this, [this]() { SaveEditableSettingsFromUi(); });

    PopulateEditableControls();
    ApplyLanguageText();
  }

  void BuildJoystickStatePanel(QVBoxLayout* root) {
    joystick_state_group_ = new QGroupBox(this);
    joystick_state_group_->setTitle("");
    joystick_state_group_->setMinimumHeight(0);
    joystick_state_group_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* group_layout = new QVBoxLayout(joystick_state_group_);
    group_layout->setContentsMargins(8, 6, 8, 6);
    group_layout->setSpacing(2);

    joystick_waiting_label_ = new QLabel(this);

    auto* state_body = new QWidget(joystick_state_group_);
    auto* state_layout = new QHBoxLayout(state_body);
    state_layout->setContentsMargins(0, 0, 0, 0);
    state_layout->setSpacing(12);

    axes_panel_widget_ = new QWidget(state_body);
    buttons_panel_widget_ = new QWidget(state_body);
    auto* axes_panel_layout = new QVBoxLayout(axes_panel_widget_);
    auto* buttons_panel_layout = new QVBoxLayout(buttons_panel_widget_);
    axes_panel_layout->setContentsMargins(0, 0, 0, 0);
    buttons_panel_layout->setContentsMargins(0, 0, 0, 0);
    axes_panel_layout->setSpacing(0);
    buttons_panel_layout->setSpacing(0);

    const JoystickStateLayoutMetrics metrics = DefaultJoystickStateLayoutMetrics();
    axes_panel_widget_->setMinimumWidth(metrics.axis_card_min_width * 2);
    buttons_panel_widget_->setMinimumWidth(metrics.button_cell_min_width * 2);
    axes_panel_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    buttons_panel_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    axes_layout_ = new QGridLayout();
    buttons_layout_ = new QGridLayout();
    axes_layout_->setHorizontalSpacing(6);
    axes_layout_->setVerticalSpacing(3);
    buttons_layout_->setHorizontalSpacing(5);
    buttons_layout_->setVerticalSpacing(3);

    axes_panel_layout->addLayout(axes_layout_, 1);
    buttons_panel_layout->addLayout(buttons_layout_, 1);
    state_layout->addWidget(axes_panel_widget_, 3);
    state_layout->addWidget(buttons_panel_widget_, 2);

    joystick_state_scroll_ = new QScrollArea(joystick_state_group_);
    joystick_state_scroll_->setWidgetResizable(true);
    joystick_state_scroll_->setFrameShape(QFrame::NoFrame);
    joystick_state_scroll_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    joystick_state_scroll_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    joystick_state_scroll_->setWidget(state_body);

    group_layout->addWidget(joystick_waiting_label_);
    group_layout->addWidget(joystick_state_scroll_, 1);
    root->addWidget(joystick_state_group_, 1);
  }

  JoyStateSnapshot CurrentJoySnapshot() const {
    std::vector<double> axes;
    axes.reserve(latest_joy_.axes.size());
    for (const float axis : latest_joy_.axes) {
      axes.push_back(static_cast<double>(axis));
    }
    return BuildJoyStateSnapshot(axes, latest_joy_.buttons);
  }

  void RefreshJoystickStatePanel() {
    if (!have_joy_) {
      joystick_waiting_label_->setText(Tr("等待 /joy 手柄输入", "Waiting for /joy joystick input"));
      return;
    }

    joystick_waiting_label_->clear();
    const JoyStateSnapshot snapshot = CurrentJoySnapshot();
    EnsureJoyStateWidgets(snapshot);

    for (std::size_t index = 0; index < snapshot.axes.size(); ++index) {
      axis_widgets_[index].name_label->setText(QString("A%1").arg(snapshot.axes[index].index));
      axis_widgets_[index].bar->SetValue(snapshot.axes[index].value);
      axis_widgets_[index].value_label->setText(QString("%1").arg(snapshot.axes[index].value, 0, 'f', 2));
    }

    for (std::size_t index = 0; index < snapshot.buttons.size(); ++index) {
      button_widgets_[index].dot->SetPressed(snapshot.buttons[index].pressed);
      button_widgets_[index].name_label->setText(QString::number(snapshot.buttons[index].index));
    }
  }

  void EnsureJoyStateWidgets(const JoyStateSnapshot& snapshot) {
    if (axis_widgets_.size() != snapshot.axes.size()) {
      ClearLayoutItems(axes_layout_);
      for (AxisStateWidgets& row : axis_widgets_) {
        delete row.container;
      }
      axis_widgets_.clear();
      axis_widgets_.reserve(snapshot.axes.size());
      const JoystickStateLayoutMetrics metrics = DefaultJoystickStateLayoutMetrics();
      for (std::size_t index = 0; index < snapshot.axes.size(); ++index) {
        AxisStateWidgets row;
        row.container = new QWidget(this);
        auto* row_layout = new QVBoxLayout(row.container);
        row.name_label = new QLabel(row.container);
        row.bar = new AxisBarWidget(row.container);
        row.value_label = new QLabel(row.container);
        row.container->setMinimumSize(metrics.axis_card_min_width, metrics.axis_card_min_height);
        row.container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        row_layout->setContentsMargins(1, 1, 1, 1);
        row_layout->setSpacing(2);
        row.name_label->setAlignment(Qt::AlignCenter);
        row.value_label->setAlignment(Qt::AlignCenter);
        row.name_label->setText(QString("A%1").arg(snapshot.axes[index].index));
        row_layout->addWidget(row.name_label);
        row_layout->addWidget(row.bar, 1);
        row_layout->addWidget(row.value_label);
        axis_widgets_.push_back(row);
      }
    }

    if (button_widgets_.size() != snapshot.buttons.size()) {
      ClearLayoutItems(buttons_layout_);
      for (ButtonStateWidgets& button : button_widgets_) {
        delete button.container;
      }
      button_widgets_.clear();
      button_widgets_.reserve(snapshot.buttons.size());
      const JoystickStateLayoutMetrics metrics = DefaultJoystickStateLayoutMetrics();
      for (std::size_t index = 0; index < snapshot.buttons.size(); ++index) {
        ButtonStateWidgets button;
        button.container = new QWidget(this);
        auto* cell_layout = new QVBoxLayout(button.container);
        button.dot = new ButtonDotWidget(button.container);
        button.name_label = new QLabel(button.container);
        button.container->setMinimumSize(metrics.button_cell_min_width, metrics.button_cell_min_height);
        button.container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        cell_layout->setContentsMargins(1, 1, 1, 1);
        cell_layout->setSpacing(2);
        button.name_label->setAlignment(Qt::AlignCenter);
        cell_layout->addWidget(button.name_label);
        cell_layout->addStretch(1);
        cell_layout->addWidget(button.dot, 0, Qt::AlignHCenter);
        cell_layout->addStretch(1);
        button_widgets_.push_back(button);
      }
    }

    RelayoutJoyStateWidgets();
  }

  void RelayoutJoyStateWidgets() {
    if (axes_layout_ == nullptr || buttons_layout_ == nullptr) {
      return;
    }

    ClearLayoutItems(axes_layout_);
    ClearLayoutItems(buttons_layout_);

    const JoystickStateLayoutMetrics metrics = DefaultJoystickStateLayoutMetrics();
    const int axes_width = std::max(metrics.axis_card_min_width, axes_panel_widget_ == nullptr ? 0 : axes_panel_widget_->width() - 8);
    const int buttons_width = std::max(metrics.button_cell_min_width, buttons_panel_widget_ == nullptr ? 0 : buttons_panel_widget_->width() - 8);
    const std::size_t axis_columns = AxisGridColumnsForWidth(axis_widgets_.size(), axes_width);
    const std::size_t button_columns = ButtonGridColumnsForWidth(button_widgets_.size(), buttons_width);

    for (std::size_t index = 0; index < axis_layout_column_count_; ++index) {
      axes_layout_->setColumnStretch(static_cast<int>(index), 0);
    }
    for (std::size_t index = 0; index < axis_widgets_.size(); ++index) {
      const int layout_row = static_cast<int>(index / axis_columns);
      const int layout_col = static_cast<int>(index % axis_columns);
      axes_layout_->addWidget(axis_widgets_[index].container, layout_row, layout_col);
    }
    for (std::size_t index = 0; index < axis_columns; ++index) {
      axes_layout_->setColumnStretch(static_cast<int>(index), 1);
    }
    axis_layout_column_count_ = axis_columns;

    for (std::size_t index = 0; index < button_layout_column_count_; ++index) {
      buttons_layout_->setColumnStretch(static_cast<int>(index), 0);
    }
    for (std::size_t index = 0; index < button_widgets_.size(); ++index) {
      const int layout_row = static_cast<int>(index / button_columns);
      const int layout_col = static_cast<int>(index % button_columns);
      buttons_layout_->addWidget(button_widgets_[index].container, layout_row, layout_col);
    }
    for (std::size_t index = 0; index < button_columns; ++index) {
      buttons_layout_->setColumnStretch(static_cast<int>(index), 1);
    }
    button_layout_column_count_ = button_columns;
  }

  static void ClearLayoutItems(QLayout* layout) {
    if (layout == nullptr) {
      return;
    }
    while (QLayoutItem* item = layout->takeAt(0)) {
      delete item;
    }
  }

  void BuildVelocityEditor(QVBoxLayout* root) {
    velocity_group_ = new QGroupBox(this);
    auto* layout = new QGridLayout(velocity_group_);
    layout->setContentsMargins(6, 8, 6, 6);
    layout->setHorizontalSpacing(8);
    layout->setVerticalSpacing(3);
    velocity_group_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    axis_header_label_ = new QLabel(this);
    max_output_header_label_ = new QLabel(this);
    linear_x_label_ = new QLabel(this);
    linear_y_label_ = new QLabel(this);
    linear_z_label_ = new QLabel(this);
    angular_yaw_label_ = new QLabel(this);
    linear_x_axis_spin_ = MakeAxisSpinBox();
    linear_y_axis_spin_ = MakeAxisSpinBox();
    linear_z_axis_spin_ = MakeAxisSpinBox();
    angular_yaw_axis_spin_ = MakeAxisSpinBox();
    linear_x_spin_ = MakeVelocitySpinBox(" m/s");
    linear_y_spin_ = MakeVelocitySpinBox(" m/s");
    linear_z_spin_ = MakeVelocitySpinBox(" m/s");
    angular_yaw_spin_ = MakeVelocitySpinBox(" rad/s");

    layout->addWidget(axis_header_label_, 0, 1);
    layout->addWidget(max_output_header_label_, 0, 2);
    layout->addWidget(linear_x_label_, 1, 0);
    layout->addWidget(linear_x_axis_spin_, 1, 1);
    layout->addWidget(linear_x_spin_, 1, 2);
    layout->addWidget(linear_y_label_, 2, 0);
    layout->addWidget(linear_y_axis_spin_, 2, 1);
    layout->addWidget(linear_y_spin_, 2, 2);
    layout->addWidget(linear_z_label_, 3, 0);
    layout->addWidget(linear_z_axis_spin_, 3, 1);
    layout->addWidget(linear_z_spin_, 3, 2);
    layout->addWidget(angular_yaw_label_, 4, 0);
    layout->addWidget(angular_yaw_axis_spin_, 4, 1);
    layout->addWidget(angular_yaw_spin_, 4, 2);
    layout->setColumnStretch(2, 1);
    root->addWidget(velocity_group_, 0);

    const auto connect_axis_spin = [this](QSpinBox* spin) {
      connect(spin, QOverload<int>::of(&QSpinBox::valueChanged), this, [this](int) { ApplyAxisMappingFromUi(); });
    };
    const auto connect_spin = [this](QDoubleSpinBox* spin) {
      connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) { ApplyVelocityLimitsFromUi(); });
    };
    connect_axis_spin(linear_x_axis_spin_);
    connect_axis_spin(linear_y_axis_spin_);
    connect_axis_spin(linear_z_axis_spin_);
    connect_axis_spin(angular_yaw_axis_spin_);
    connect_spin(linear_x_spin_);
    connect_spin(linear_y_spin_);
    connect_spin(linear_z_spin_);
    connect_spin(angular_yaw_spin_);
  }

  QDoubleSpinBox* MakeVelocitySpinBox(const char* suffix) {
    auto* spin = new QDoubleSpinBox(this);
    spin->setRange(0.0, 100.0);
    spin->setDecimals(3);
    spin->setSingleStep(0.05);
    spin->setSuffix(suffix);
    return spin;
  }

  QSpinBox* MakeAxisSpinBox() {
    auto* spin = new QSpinBox(this);
    spin->setRange(-1, 63);
    spin->setSingleStep(1);
    spin->setMinimumWidth(72);
    return spin;
  }

  void BuildActionEditor(QVBoxLayout* root) {
    action_group_ = new QGroupBox(this);
    action_group_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    auto* layout = new QGridLayout(action_group_);
    layout->setContentsMargins(5, 6, 5, 5);
    layout->setHorizontalSpacing(6);
    layout->setVerticalSpacing(3);
    const ActionEditorColumnWidths widths = DefaultActionEditorColumnWidths();
    layout->setColumnMinimumWidth(1, widths.service_min_width);
    layout->setColumnMinimumWidth(2, widths.buttons_min_width);
    layout->setColumnMinimumWidth(3, widths.keyboard_width);
    layout->setColumnMinimumWidth(4, widths.small_button_width);
    layout->setColumnMinimumWidth(5, widths.small_button_width);
    layout->setColumnStretch(1, 5);
    layout->setColumnStretch(2, 2);
    layout->setColumnStretch(3, 0);
    layout->setColumnStretch(4, 0);
    layout->setColumnStretch(5, 0);

    action_slot_header_label_ = new QLabel(this);
    action_service_header_label_ = new QLabel(this);
    action_buttons_header_label_ = new QLabel(this);
    action_keyboard_header_label_ = new QLabel(this);
    layout->addWidget(action_slot_header_label_, 0, 0);
    layout->addWidget(action_service_header_label_, 0, 1);
    layout->addWidget(action_buttons_header_label_, 0, 2);
    layout->addWidget(action_keyboard_header_label_, 0, 3);

    action_rows_.clear();
    action_rows_.reserve(kEditableActionSlotCount);
    for (std::size_t index = 0; index < kEditableActionSlotCount; ++index) {
      ActionRowWidgets row;
      row.name_label = new QLabel(this);
      row.service_edit = new QLineEdit(this);
      row.buttons_edit = new QLineEdit(this);
      row.keyboard_edit = new QLineEdit(this);
      row.capture_button = new QPushButton(this);
      row.clear_button = new QPushButton(this);
      row.keyboard_edit->setMaxLength(1);
      row.buttons_edit->setPlaceholderText("4,0");
      row.service_edit->setPlaceholderText("/joy/actionN");
      row.name_label->setMinimumWidth(54);
      row.name_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
      row.service_edit->setMinimumWidth(widths.service_min_width);
      row.service_edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      row.service_edit->setMinimumHeight(22);
      row.buttons_edit->setMinimumWidth(widths.buttons_min_width);
      row.buttons_edit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
      row.buttons_edit->setMinimumHeight(22);
      row.keyboard_edit->setFixedWidth(widths.keyboard_width);
      row.keyboard_edit->setMinimumHeight(22);
      row.capture_button->setMinimumWidth(widths.small_button_width);
      row.capture_button->setMaximumWidth(widths.small_button_width);
      row.capture_button->setMinimumHeight(22);
      row.clear_button->setMinimumWidth(widths.small_button_width);
      row.clear_button->setMaximumWidth(widths.small_button_width);
      row.clear_button->setMinimumHeight(22);

      const int layout_row = static_cast<int>(index) + 1;
      layout->addWidget(row.name_label, layout_row, 0);
      layout->addWidget(row.service_edit, layout_row, 1);
      layout->addWidget(row.buttons_edit, layout_row, 2);
      layout->addWidget(row.keyboard_edit, layout_row, 3);
      layout->addWidget(row.capture_button, layout_row, 4);
      layout->addWidget(row.clear_button, layout_row, 5);

      connect(row.capture_button, &QPushButton::clicked, this, [this, index]() { CaptureActionButtons(index); });
      connect(row.clear_button, &QPushButton::clicked, this, [this, index]() { ClearActionButtons(index); });

      action_rows_.push_back(row);
    }

    root->addWidget(action_group_, 0);
  }

  void PopulateEditableControls() {
    linear_x_axis_spin_->blockSignals(true);
    linear_y_axis_spin_->blockSignals(true);
    linear_z_axis_spin_->blockSignals(true);
    angular_yaw_axis_spin_->blockSignals(true);
    linear_x_axis_spin_->setValue(editable_settings_.axis_mapping.linear_x);
    linear_y_axis_spin_->setValue(editable_settings_.axis_mapping.linear_y);
    linear_z_axis_spin_->setValue(editable_settings_.axis_mapping.linear_z);
    angular_yaw_axis_spin_->setValue(editable_settings_.axis_mapping.angular_yaw);
    linear_x_axis_spin_->blockSignals(false);
    linear_y_axis_spin_->blockSignals(false);
    linear_z_axis_spin_->blockSignals(false);
    angular_yaw_axis_spin_->blockSignals(false);

    const MotionScale limits = CurrentMaxVelocityLimits(editable_settings_);
    linear_x_spin_->blockSignals(true);
    linear_y_spin_->blockSignals(true);
    linear_z_spin_->blockSignals(true);
    angular_yaw_spin_->blockSignals(true);
    linear_x_spin_->setValue(limits.linear_x);
    linear_y_spin_->setValue(limits.linear_y);
    linear_z_spin_->setValue(limits.linear_z);
    angular_yaw_spin_->setValue(limits.angular_yaw);
    linear_x_spin_->blockSignals(false);
    linear_y_spin_->blockSignals(false);
    linear_z_spin_->blockSignals(false);
    angular_yaw_spin_->blockSignals(false);

    for (std::size_t index = 0; index < action_rows_.size() && index < editable_settings_.action_slots.size(); ++index) {
      const GuiActionSlotConfig& slot = editable_settings_.action_slots[index];
      ActionRowWidgets& row = action_rows_[index];
      row.name_label->setText(QString::fromStdString(slot.name));
      row.service_edit->setText(QString::fromStdString(slot.service_name));
      row.buttons_edit->setText(ButtonsText(slot.buttons));
      row.keyboard_edit->setText(QString::fromStdString(slot.keyboard_key));
    }
  }

  MotionScale VelocityLimitsFromUi() const {
    MotionScale limits;
    limits.linear_x = linear_x_spin_->value();
    limits.linear_y = linear_y_spin_->value();
    limits.linear_z = linear_z_spin_->value();
    limits.angular_yaw = angular_yaw_spin_->value();
    return limits;
  }

  JoystickAxisMapping AxisMappingFromUi() const {
    JoystickAxisMapping mapping;
    mapping.linear_x = linear_x_axis_spin_->value();
    mapping.linear_y = linear_y_axis_spin_->value();
    mapping.linear_z = linear_z_axis_spin_->value();
    mapping.angular_yaw = angular_yaw_axis_spin_->value();
    return mapping;
  }

  void ApplyAxisMappingFromUi() {
    const JoystickAxisMapping mapping = AxisMappingFromUi();
    config_.axes.linear_x = mapping.linear_x;
    config_.axes.linear_y = mapping.linear_y;
    config_.axes.linear_z = mapping.linear_z;
    config_.axes.angular_yaw = mapping.angular_yaw;
    SetStatus(Tr("轴映射已应用，尚未保存", "Axis mapping applied, not saved"));
  }

  void ApplyVelocityLimitsFromUi() {
    EditableGuiSettings settings = editable_settings_;
    ApplyMaxVelocityLimits(VelocityLimitsFromUi(), &settings);
    config_.joystick_scale = settings.joystick_scale;
    config_.keyboard_scale = settings.keyboard_scale;
    SetStatus(Tr("速度已应用，尚未保存", "Speed limits applied, not saved"));
  }

  EditableGuiSettings CollectEditableSettingsFromUi() const {
    EditableGuiSettings settings = editable_settings_;
    settings.language = language_;
    settings.axis_mapping = AxisMappingFromUi();
    ApplyMaxVelocityLimits(VelocityLimitsFromUi(), &settings);
    for (std::size_t index = 0; index < action_rows_.size() && index < settings.action_slots.size(); ++index) {
      const ActionRowWidgets& row = action_rows_[index];
      settings.action_slots[index].service_name = row.service_edit->text().trimmed().toStdString();
      settings.action_slots[index].buttons = ParseButtonsText(row.buttons_edit->text());
      settings.action_slots[index].keyboard_key = row.keyboard_edit->text().left(1).toLower().toStdString();
    }
    return settings;
  }

  void SaveEditableSettingsFromUi() {
    if (config_.config_path.empty()) {
      SetStatus(Tr("未配置 gui.config_path，无法写回 YAML", "gui.config_path is not set; cannot save YAML"));
      return;
    }

    try {
      EditableGuiSettings settings = CollectEditableSettingsFromUi();
      SaveEditableGuiSettingsToYaml(config_.config_path, settings);
      editable_settings_ = settings;
      ApplyEditableSettingsToRuntime(editable_settings_);
      RebuildActionClients();
      SetStatus(Tr("配置已保存并应用", "Configuration saved and applied"));
      RefreshUi("Idle");
    } catch (const std::exception& error) {
      SetStatus(Tr("保存失败：", "Save failed: ") + QString::fromStdString(error.what()));
    }
  }

  void ToggleLanguage() {
    language_ = language_ == "en" ? "zh" : "en";
    editable_settings_.language = language_;
    ApplyLanguageText();
    SetStatus(Tr("语言已切换，点击保存写入配置", "Language changed; save to persist"));
    RefreshUi("Idle");
  }

  void CaptureActionButtons(std::size_t index) {
    if (index >= action_rows_.size()) {
      return;
    }
    const std::vector<int> buttons = CurrentPressedButtons();
    action_rows_[index].buttons_edit->setText(ButtonsText(buttons));
    SetStatus(buttons.empty() ? Tr("当前没有按下的手柄按钮", "No joystick buttons are currently pressed")
                              : Tr("已捕获当前手柄按钮", "Captured current joystick buttons"));
  }

  void ClearActionButtons(std::size_t index) {
    if (index >= action_rows_.size()) {
      return;
    }
    action_rows_[index].buttons_edit->clear();
    SetStatus(Tr("已清空按钮绑定，保存后生效", "Button binding cleared; save to apply"));
  }

  std::vector<int> CurrentPressedButtons() const {
    std::vector<int> buttons;
    if (!have_joy_) {
      return buttons;
    }
    for (int i = 0; i < static_cast<int>(latest_joy_.buttons.size()); ++i) {
      if (latest_joy_.buttons[i] != 0) {
        buttons.push_back(i);
      }
    }
    return buttons;
  }

  QString Tr(const char* zh, const char* en) const { return language_ == "en" ? QString(en) : QString(zh); }

  QString ModeText(const std::string& mode) const {
    if (mode == "Joystick") {
      return Tr("手柄", "Joystick");
    }
    if (mode == "Keyboard") {
      return Tr("键盘", "Keyboard");
    }
    return Tr("空闲", "Idle");
  }

  void SetStatus(const QString& message) {
    if (save_status_label_ != nullptr) {
      save_status_label_->setText(message);
    }
  }

  void ApplyLanguageText() {
    setWindowTitle(Tr("AUSIM 遥控控制", "AUSIM Remote Control"));
    mode_title_label_->setText(Tr("输入来源", "Input Source"));
    hint_label_->setText(Tr("聚焦此窗口后可用键盘控制：WASD 平移，R/F 升降，J/L 偏航，空格停止。",
                            "Focus this window for keyboard control. WASD move, R/F up-down, J/L yaw, Space stop."));

    joystick_state_group_->setTitle("");
    if (!have_joy_) {
      joystick_waiting_label_->setText(Tr("等待 /joy 手柄输入", "Waiting for /joy joystick input"));
    }

    velocity_group_->setTitle(Tr("cmd_vel 轴映射与最大输出", "cmd_vel Axis Mapping and Max Output"));
    axis_header_label_->setText(Tr("手柄轴", "Axis"));
    max_output_header_label_->setText(Tr("最大输出", "Max Output"));
    linear_x_label_->setText("linear.x");
    linear_y_label_->setText("linear.y");
    linear_z_label_->setText("linear.z");
    angular_yaw_label_->setText("angular.z");

    action_group_->setTitle(Tr("手柄到 Action 映射（默认 6 槽）", "Joystick to Action Mapping (6 Slots)"));
    action_slot_header_label_->setText(Tr("槽位", "Slot"));
    action_service_header_label_->setText("service");
    action_buttons_header_label_->setText(Tr("手柄按钮", "Buttons"));
    action_keyboard_header_label_->setText(Tr("键盘键", "Key"));
    for (ActionRowWidgets& row : action_rows_) {
      row.capture_button->setText(Tr("取", "Cap"));
      row.clear_button->setText(Tr("清", "Clr"));
    }

    language_button_->setText(language_ == "en" ? "中文" : "English");
    save_button_->setText(Tr("应用并保存", "Apply && Save"));
    action_history_title_label_->setText(Tr("Action 历史", "Action History"));
  }

  void SetupRosInterfaces() {
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(config_.cmd_vel_topic, 10);
    joy_subscription_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        config_.joy_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Joy::SharedPtr message) { OnJoy(std::move(message)); });

    RebuildActionClients();
  }

  void RebuildActionClients() {
    action_clients_.clear();
    action_combo_active_.clear();
    for (const auto& [action_name, binding] : config_.actions) {
      action_clients_.emplace(action_name, node_->create_client<std_srvs::srv::Trigger>(binding.service_name));
      action_combo_active_.emplace(action_name, false);
    }
  }

  void OnJoy(sensor_msgs::msg::Joy::SharedPtr message) {
    latest_joy_ = *message;
    have_joy_ = true;
    last_joy_time_ = std::chrono::steady_clock::now();

    for (const auto& [action_name, binding] : config_.actions) {
      const bool pressed = !binding.buttons.empty() && ButtonsPressed(latest_joy_, binding.buttons);
      bool& was_active = action_combo_active_[action_name];
      if (pressed && !was_active) {
        TriggerAction(action_name, "joystick");
      }
      was_active = pressed;
    }
  }

  void OnTimer() {
    rclcpp::spin_some(node_);
    if (!rclcpp::ok()) {
      QApplication::quit();
      return;
    }

    geometry_msgs::msg::Twist command;
    const auto now = std::chrono::steady_clock::now();
    const bool joystick_active = have_joy_ && std::chrono::duration<double>(now - last_joy_time_).count() <= config_.joy_timeout_seconds;

    if (!joystick_active) {
      for (auto& [name, active] : action_combo_active_) {
        (void)name;
        active = false;
      }
    }

    std::string mode = "Idle";
    if (now < suppress_motion_until_) {
      command = geometry_msgs::msg::Twist();
      mode = joystick_active ? "Joystick" : (config_.keyboard_enabled && model_.PressedKeysText() != "None" ? "Keyboard" : "Idle");
    } else if (joystick_active) {
      command = BuildJoyTwist(latest_joy_);
      mode = "Joystick";
    } else if (config_.keyboard_enabled) {
      command = model_.BuildKeyboardTwist(config_.keyboard_scale);
      mode = model_.PressedKeysText() == "None" ? "Idle" : "Keyboard";
    }

    latest_command_ = command;
    cmd_vel_publisher_->publish(command);
    RefreshUi(mode);
  }

  void TriggerAction(const std::string& action_name, const char* source) {
    const auto binding_it = config_.actions.find(action_name);
    const auto client_it = action_clients_.find(action_name);
    if (binding_it == config_.actions.end() || client_it == action_clients_.end() || !client_it->second) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (last_action_time_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(now - last_action_time_).count() < config_.command_cooldown_seconds) {
      return;
    }

    last_action_time_ = now;
    suppress_motion_until_ = now + config_.motion_suppress_duration;
    PublishZeroCommand();

    const std::string service_name = binding_it->second.service_name;
    auto client = client_it->second;
    if (!client->service_is_ready()) {
      model_.RecordAction(CurrentTimeText(), source, action_name, service_name, "service not ready");
      RefreshActionHistory();
      return;
    }

    model_.RecordAction(CurrentTimeText(), source, action_name, service_name, "sent");
    RefreshActionHistory();
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(
        request, [this, action_name, service_name, source_text = std::string(source)](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          try {
            const auto response = future.get();
            const std::string result = response == nullptr ? "no response" : (response->success ? "success: " : "failed: ") + response->message;
            model_.RecordAction(CurrentTimeText(), source_text, action_name, service_name, result);
          } catch (const std::exception& error) {
            model_.RecordAction(CurrentTimeText(), source_text, action_name, service_name, std::string("error: ") + error.what());
          }
          RefreshActionHistory();
        });
  }

  void PublishZeroCommand() {
    latest_command_ = geometry_msgs::msg::Twist();
    cmd_vel_publisher_->publish(latest_command_);
  }

  std::optional<std::string> KeyboardActionForKey(char key) const {
    for (const auto& [action_name, binding] : config_.actions) {
      if (binding.keyboard_key != '\0' && binding.keyboard_key == key) {
        return action_name;
      }
    }
    return std::nullopt;
  }

  geometry_msgs::msg::Twist BuildJoyTwist(const sensor_msgs::msg::Joy& message) const {
    geometry_msgs::msg::Twist command;
    if (config_.require_enable_button && !ButtonPressed(message, config_.enable_button)) {
      return command;
    }

    command.linear.x = AxisValue(message, config_.axes.linear_x, config_.joystick_scale.linear_x);
    command.linear.y = AxisValue(message, config_.axes.linear_y, config_.joystick_scale.linear_y);
    command.linear.z = AxisValue(message, config_.axes.linear_z, config_.joystick_scale.linear_z);
    command.angular.z = AxisValue(message, config_.axes.angular_yaw, config_.joystick_scale.angular_yaw);
    return command;
  }

  bool ButtonPressed(const sensor_msgs::msg::Joy& message, int button_index) const {
    return button_index >= 0 && button_index < static_cast<int>(message.buttons.size()) && message.buttons[button_index] != 0;
  }

  bool ButtonsPressed(const sensor_msgs::msg::Joy& message, const std::vector<int>& button_indices) const {
    if (button_indices.empty()) {
      return false;
    }
    return std::all_of(button_indices.begin(), button_indices.end(),
                       [this, &message](int button_index) { return ButtonPressed(message, button_index); });
  }

  double AxisValue(const sensor_msgs::msg::Joy& message, int axis_index, double scale) const {
    if (axis_index < 0 || axis_index >= static_cast<int>(message.axes.size())) {
      return 0.0;
    }
    return ApplyDeadzone(message.axes[axis_index], config_.deadzone) * scale;
  }

  static bool IsMovementKey(char key) {
    switch (key) {
      case 'w':
      case 's':
      case 'a':
      case 'd':
      case 'r':
      case 'f':
      case 'j':
      case 'l':
        return true;
      default:
        return false;
    }
  }

  void RefreshUi(const std::string& mode) {
    mode_label_->setText(ModeText(mode));
    RefreshJoystickStatePanel();
    RefreshActionHistory();
  }

  void RefreshActionHistory() {
    const std::vector<std::string> entries = model_.ActionHistoryText();
    history_list_->clear();
    for (const std::string& entry : entries) {
      history_list_->addItem(QString::fromStdString(entry));
    }
    if (history_list_->count() > 0) {
      history_list_->scrollToBottom();
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  GuiConfig config_;
  EditableGuiSettings editable_settings_;
  GuiControlModel model_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  std::unordered_map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> action_clients_;
  std::unordered_map<std::string, bool> action_combo_active_;

  QTimer* publish_timer_ = nullptr;
  QLabel* mode_label_ = nullptr;
  QLabel* mode_title_label_ = nullptr;
  QLabel* hint_label_ = nullptr;
  QGroupBox* joystick_state_group_ = nullptr;
  QLabel* joystick_waiting_label_ = nullptr;
  QScrollArea* joystick_state_scroll_ = nullptr;
  QWidget* axes_panel_widget_ = nullptr;
  QWidget* buttons_panel_widget_ = nullptr;
  QGridLayout* axes_layout_ = nullptr;
  QGridLayout* buttons_layout_ = nullptr;
  std::size_t axis_layout_column_count_ = 0;
  std::size_t button_layout_column_count_ = 0;
  std::vector<AxisStateWidgets> axis_widgets_;
  std::vector<ButtonStateWidgets> button_widgets_;
  QGroupBox* velocity_group_ = nullptr;
  QLabel* axis_header_label_ = nullptr;
  QLabel* max_output_header_label_ = nullptr;
  QLabel* linear_x_label_ = nullptr;
  QLabel* linear_y_label_ = nullptr;
  QLabel* linear_z_label_ = nullptr;
  QLabel* angular_yaw_label_ = nullptr;
  QSpinBox* linear_x_axis_spin_ = nullptr;
  QSpinBox* linear_y_axis_spin_ = nullptr;
  QSpinBox* linear_z_axis_spin_ = nullptr;
  QSpinBox* angular_yaw_axis_spin_ = nullptr;
  QDoubleSpinBox* linear_x_spin_ = nullptr;
  QDoubleSpinBox* linear_y_spin_ = nullptr;
  QDoubleSpinBox* linear_z_spin_ = nullptr;
  QDoubleSpinBox* angular_yaw_spin_ = nullptr;
  QGroupBox* action_group_ = nullptr;
  QLabel* action_slot_header_label_ = nullptr;
  QLabel* action_service_header_label_ = nullptr;
  QLabel* action_buttons_header_label_ = nullptr;
  QLabel* action_keyboard_header_label_ = nullptr;
  std::vector<ActionRowWidgets> action_rows_;
  QPushButton* language_button_ = nullptr;
  QPushButton* save_button_ = nullptr;
  QLabel* save_status_label_ = nullptr;
  QLabel* action_history_title_label_ = nullptr;
  QListWidget* history_list_ = nullptr;
  std::string language_ = "zh";

  bool have_joy_ = false;
  sensor_msgs::msg::Joy latest_joy_;
  std::chrono::steady_clock::time_point last_joy_time_{};
  geometry_msgs::msg::Twist latest_command_;
  std::chrono::steady_clock::time_point suppress_motion_until_{};
  std::chrono::steady_clock::time_point last_action_time_{};
};

}  // namespace remote_control

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::vector<std::string> qt_arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char*> qt_argv;
  qt_argv.reserve(qt_arguments.size());
  for (std::string& argument : qt_arguments) {
    qt_argv.push_back(argument.data());
  }
  int qt_argc = static_cast<int>(qt_argv.size());

  QApplication app(qt_argc, qt_argv.data());
  auto node = std::make_shared<rclcpp::Node>("remote_control_gui_node");
  remote_control::RemoteControlGuiWindow window(node);
  window.show();

  const int result = app.exec();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
