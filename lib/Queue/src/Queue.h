//
// Copyright (c) 2013 Juan Palacios juan.palacios.puyana@gmail.com
// Subject to the BSD 2-Clause License
// - see < http://opensource.org/licenses/BSD-2-Clause>
//

#ifndef CONCURRENT_QUEUE_
#define CONCURRENT_QUEUE_

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

template <typename T> class Queue {
  public:
	T pop() {
		std::unique_lock<std::mutex> mlock(mutex_);
		while (queue_.empty()) {
			cond_.wait(mlock);
		}
		auto val = queue_.front();
		queue_.pop();
		return val;
	}

	void pop(T &item) {
		std::unique_lock<std::mutex> mlock(mutex_);
		while (queue_.empty()) {
			cond_.wait(mlock);
		}
		item = queue_.front();
		queue_.pop();
	}

	void push(const T &item) {
		std::unique_lock<std::mutex> mlock(mutex_);
		queue_.push(item);
		mlock.unlock();
		cond_.notify_one();
	}
	void push(const T *items, int len) {
		std::unique_lock<std::mutex> mlock(mutex_);
		for (int i = 0; i < len; i++) {
			queue_.push(items[i]);
		}
		mlock.unlock();
		cond_.notify_one();
	}
	bool empty() {
		std::unique_lock<std::mutex> mlock(mutex_);
		bool res = queue_.empty();
		mlock.unlock();
		return res;
	}
	Queue() = default;
	Queue(const Queue &) = delete;			  // disable copying
	Queue &operator=(const Queue &) = delete; // disable assignment

  private:
	std::queue<T> queue_;
	std::mutex mutex_;
	std::condition_variable cond_;
};

#endif