#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <optional>

namespace humanoid_slam{
template <typename T>
class BlockingQueue{
public:
    BlockingQueue(const BlockingQueue&) = delete;
    BlockingQueue& operator=(const BlockingQueue&) = delete;
    
    explicit BlockingQueue( const size_t nQueueMaxSize = 0 )
        : m_nQueueMaxSize(nQueueMaxSize){} // 0 = infinity

    void Push( T t ){
        std::unique_lock lock(m_mutex);
        while( m_nQueueMaxSize && m_queue.size()>= m_nQueueMaxSize ){
            m_productCondVar.wait(lock);
            if(m_done){
                return;
            }
        }
        m_queue.push(std::move(t));
        m_customCondVar.notify_one();
    }

    T Pop(){
        std::unique_lock lock(m_mutex);
        while( m_queue.empty() ){
            m_customCondVar.wait(lock);
            if(m_done){
                return T{};
            }
        }
        T t = std::move(m_queue.front());
        m_queue.pop();
        m_productCondVar.notify_one();
        return t;
    }

    size_t Size(){
        std::unique_lock lock(m_mutex);
        return m_queue.size();
    }

    void Done(){
        std::unique_lock lock(m_mutex);
        m_done = true;
        m_customCondVar.notify_all();
        m_productCondVar.notify_all();
    }

    bool IsDone(){
        std::unique_lock lock(m_mutex);
        return m_done;
    }

private:
    std::queue<T> m_queue;
    size_t m_nQueueMaxSize = 0;
    std::mutex m_mutex;
    std::condition_variable m_productCondVar;
    std::condition_variable m_customCondVar;
    std::atomic<bool> m_done{false};
};

}// namespace humanoid_slam