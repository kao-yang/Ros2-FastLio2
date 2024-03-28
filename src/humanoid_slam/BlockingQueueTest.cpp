#include "humanoid_slam/BlockingQueue.h"
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

namespace humanoid_slam{
namespace{

TEST(BlockingQueueTest, PushPopTest){
    BlockingQueue<std::unique_ptr<int>> queue;
    ASSERT_EQ((size_t)0, queue.Size());
    queue.Push(std::unique_ptr<int>(new int(11)));
    ASSERT_EQ((size_t)1, queue.Size());
    queue.Push(std::unique_ptr<int>(new int(22)));
    ASSERT_EQ((size_t)2, queue.Size());
    ASSERT_EQ(11, *queue.Pop());
    ASSERT_EQ((size_t)1, queue.Size());
    ASSERT_EQ(22, *queue.Pop());
    ASSERT_EQ((size_t)0, queue.Size());
}

TEST(BlockingQueueTest, PushBlockTest){
    BlockingQueue<std::unique_ptr<int>> queue(1);
    ASSERT_EQ((size_t)0, queue.Size());
    queue.Push(std::make_unique<int>(11));
    ASSERT_EQ((size_t)1, queue.Size());
    int nPush = 22;
    std::thread thread([&queue, &nPush]{ queue.Push( std::make_unique<int>(nPush) ); });
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ((size_t)1, queue.Size());
    ASSERT_EQ(11, *queue.Pop());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ((size_t)1, queue.Size());
    ASSERT_EQ(22, *queue.Pop());
    thread.join();
}

TEST(BlockingQueueTest, PopBlockTest){
    BlockingQueue<std::unique_ptr<int>> queue;
    ASSERT_EQ((size_t)0, queue.Size());
    int nPop = 0;
    std::thread thread([&queue, &nPop]{ nPop = *queue.Pop(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    queue.Push(std::make_unique<int>(11));
    thread.join();
    ASSERT_EQ((size_t)0, queue.Size());
    EXPECT_EQ(11, nPop);
}

TEST(BlockingQueueTest, DonePopTest){
    BlockingQueue<int> queue(1);
    ASSERT_EQ((size_t)0, queue.Size());
    int nPop = 0;
    std::thread thread([&queue, &nPop]{ nPop = queue.Pop(); });
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    queue.Done();
    thread.join();
    ASSERT_EQ(true, queue.IsDone());
}

TEST(BlockingQueueTest, DonePushTest){
    BlockingQueue<int> queue(1);
    ASSERT_EQ((size_t)0, queue.Size());
    queue.Push(11);
    ASSERT_EQ((size_t)1, queue.Size());
    std::thread thread([&queue]{ queue.Push(22); });
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    queue.Done();
    thread.join();
    ASSERT_EQ(true, queue.IsDone());
    ASSERT_EQ((size_t)1, queue.Size());
}

}
} // namespace humanoid_slam