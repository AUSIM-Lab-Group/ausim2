#include "./data_board.h"
#include "./security_data.h"

#include <gtest/gtest.h>
#include <thread>

using namespace aimrte::db;

// Define a struct for testing
struct TestData {
  int x;
  double y;
  char z;
};

// Define operator== for TestData struct
bool operator==(const TestData& lhs, const TestData& rhs) { return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z; }

TEST(SecurityDataTest, SupportedType_Int) {
  SecurityData<int> atomicInt;

  atomicInt = 5;
  EXPECT_EQ(atomicInt(), 5);

  atomicInt = 10;
  EXPECT_EQ(atomicInt(), 10);
}
//
TEST(SecurityDataTest, SupportedType_Struct) {
  SecurityData<TestData> atomicStruct;

  TestData data1{1, 2.5, 'a'};
  atomicStruct = data1;
  EXPECT_EQ(atomicStruct().x, data1.x);
  EXPECT_EQ(atomicStruct().y, data1.y);
  EXPECT_EQ(atomicStruct().z, data1.z);

  TestData data2{10, 3.14, 'z'};
  atomicStruct = data2;
  EXPECT_EQ(atomicStruct().x, data2.x);
  EXPECT_EQ(atomicStruct().y, data2.y);
  EXPECT_EQ(atomicStruct().z, data2.z);
}

TEST(SecurityDataTest, UnsupportedType) {
  SecurityData<std::string> atomicPtr;

  atomicPtr = "hello";
  EXPECT_EQ(atomicPtr(), "hello");
}

enum class Tags {
  ArmTag = 1,
  LegTag = 2,
  WheelTag = 3,
};

TEST(ApiDataBoardManagerTest, ApiDataBoardManager) {
  auto int_data = DataBoard().Get<int, Permission::ReadWrite>(Tags::ArmTag);  // 等效于 std::atomic<int>
  int_data = 10;
  EXPECT_EQ(int_data(), 10);

  int_data.LockForWrite([&](auto& x) { x = 20; });
  EXPECT_EQ(int_data(), 20);

  int_data.LockForRead([&](auto& x) { EXPECT_EQ(x, 20); });

  auto read_int_data = DataBoard().Get<int>(Tags::ArmTag);
  EXPECT_EQ(read_int_data(), 20);
  // 只读数据无法写入，编译时就会报错
  // int_data.LockForWrite([&](auto& x) { x = 30; });

  // 在g++ 13 之前，shared_ptr<int> 是不可被std::atomic<int>包裹的，在g++ 13 等效于std::atomic<std::shared_ptr<int>>
  auto shared_ptr_data = DataBoard().Get<std::shared_ptr<int>, Permission::ReadWrite>();
  shared_ptr_data = std::make_shared<int>(10);
  int* ptr = shared_ptr_data().get();
  EXPECT_EQ(*ptr, 10);

  auto struct_data = DataBoard().Get<TestData, Permission::ReadWrite>();  // 使用SecurityDataRef<T>包裹
  struct_data = TestData{10, 3.14, 'z'};
  EXPECT_EQ(struct_data().x, 10);
  EXPECT_EQ(struct_data().y, 3.14);
  EXPECT_EQ(struct_data().z, 'z');

  // 安全操作
  struct_data.LockForWrite([&](auto& x) {
    x.x = 20;
    EXPECT_EQ(x.y, 3.14);
    EXPECT_EQ(x.z, 'z');
  });
  EXPECT_EQ(struct_data().x, 20);

  // read
  auto read_struct_data = DataBoard().Get<TestData>();
  read_struct_data.LockForRead([&](auto& x) {
    EXPECT_EQ(x.x, 20);
    EXPECT_EQ(x.y, 3.14);
    EXPECT_EQ(x.z, 'z');
  });

  auto string_data = DataBoard().Get<std::string, Permission::ReadWrite>();
  string_data = "hello";
  EXPECT_EQ(string_data(), "hello");

  std::cout << DataBoard().Info() << std::endl;
}

// 测试线程安全操作
TEST(ApiDataBoardManagerTest, ApiDataBoardManager_Safe) {
  auto test_data = DataBoard().Get<TestData, Permission::ReadWrite>();
  test_data = TestData{0, 0.0, 0};
  // EXPECT_EQ(int_data(), 0);

  // 创建多个线程
  std::vector<std::thread> threads;
  for (int i = 0; i < 100; i++) {
    std::thread t([&] {
      for (int j = 0; j < 1000; j++) {
        test_data.LockForWrite([&](auto& x) {
          x.y += 1.0;
          x.x = x.y + 1;
        });
      }
    });
    threads.push_back(std::move(t));
  }

  for (auto& t : threads) {
    t.join();
  }

  EXPECT_EQ(test_data().y, 100 * 1000);
  EXPECT_EQ(test_data().x, test_data().y + 1);
}

// template <typename... Args>
// void GetIdTest(Args&&... args) {
//   std::cout << "ID for ";
//   ((std::cout << args << " "), ...);
//   std::cout << ": " << GetId(std::forward<Args>(args)...) << std::endl;
// }

// TEST(GetIdTest, GetId) {
//   // 无参测试
//   GetIdTest();

//   // 添加20个相同内容但不同类型的参数
//   GetIdTest("hello");                                          // 字符串类型
//   GetIdTest(std::string("hello"));                             // 相同内容的字符串，但类型为 std::string
//   GetIdTest(std::string_view("hello"));                        // 相同内容的字符串，但类型为 std::string_view
//   GetIdTest("hello", "hello", "hello", std::string("hello"));  // 相同内容但不同类型的字符串组合

//   GetIdTest(42);    // 整数类型
//   GetIdTest(42);    // 整数类型
//   GetIdTest(42L);   // 相同内容的长整数，但类型为长整型
//   GetIdTest(42.0);  // 浮点数类型
//   GetIdTest('a');   // 字符类型

//   GetIdTest(true);                       // 布尔类型
//   GetIdTest(false);                      // 相同内容的布尔值，但类型为 false
//   GetIdTest(nullptr);                    // 空指针类型
//   GetIdTest(reinterpret_cast<int*>(0));  // 相同内容的空指针，但类型为 void*

//   GetIdTest(42, "hello");               // 整数和字符串组合
//   GetIdTest(3.14, "world", 'a');        // 浮点数、字符串和字符组合
//   GetIdTest("foo", 10, "dsadad", 2.5);  // 字符串、整数、字符和浮点数组合
// }