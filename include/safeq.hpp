
#pragma once

#include <mutex>
#include <condition_variable>

#include <queue>
#include <utility>
#include <memory>

template<class T, int len = 3>
class SafeQueue {

	std::queue<T> q;

	std::shared_ptr<std::mutex> mtx = std::make_shared<std::mutex>();
	std::shared_ptr<std::condition_variable> cv = std::make_shared<std::condition_variable>();

	std::shared_ptr<std::condition_variable> sync_wait = std::make_shared<std::condition_variable>();
	bool finish_processing = false;
	int sync_counter = 0;

	long unsigned int queue_len = len;

	// 当只有 sync_counter==0的，时候	sync_wait才会被唤醒， 而sync_wait被唤醒后，会将finish_processing设置为true
	void DecreaseSyncCounter() {
		if (--sync_counter == 0) {
			sync_wait->notify_one();
		}
	}

public:

	typedef typename std::queue<T>::size_type size_type;

	SafeQueue(){}

	SafeQueue(const SafeQueue& other) {
		std::lock_guard<std::mutex> lock(*other.mtx);
		q = other.q;
		queue_len = other.queue_len;
	}

	SafeQueue& operator=(const SafeQueue& other) {
		if (this != &other) {
			std::lock_guard<std::mutex> lock(*other.mtx);
			std::lock_guard<std::mutex> lock2(*mtx);
			q = other.q;
			queue_len = other.queue_len;
		}
		return *this;
	}

	SafeQueue(SafeQueue&& other) noexcept {
		std::lock_guard<std::mutex> lock(*other.mtx);
		q = std::move(other.q);
		queue_len = other.queue_len;
	}

	SafeQueue& operator=(SafeQueue&& other) noexcept {
		if (this != &other) {
			std::lock_guard<std::mutex> lock(*other.mtx);
			std::lock_guard<std::mutex> lock2(*mtx);
			q = std::move(other.q);
			queue_len = other.queue_len;
		}
		return *this;
	}

	~SafeQueue() {
		Finish();
	}

	void Produce(T&& item) { //	&& 表示右值引用， 可以避免拷贝

		std::lock_guard<std::mutex> lock(*mtx);

		if (q.size() < queue_len){
			q.push(std::move(item)); 
			cv->notify_one();
		}
		else {
			q.pop();
			q.push(std::move(item));
			cv->notify_one();
		}

	}

	size_type size() {

		std::lock_guard<std::mutex> lock(*mtx);

		return q.size();

	}

	[[nodiscard]]  //	异步消费	
	bool Consume(T& item) {

		std::lock_guard<std::mutex> lock(*mtx);

		if (q.empty()) {
			return false;
		}

		item = std::move(q.front());
		q.pop();

		return true;

	}

	[[nodiscard]]
	bool ConsumeSync(T& item) {

		std::unique_lock<std::mutex> lock(*mtx);

		sync_counter++;

		//	
		cv->wait(lock, [&] {	
			return !q.empty() || finish_processing; 
		});

		if (q.empty()) {  
			DecreaseSyncCounter();
			return false;
		}

		item = std::move(q.front());
		q.pop();

		DecreaseSyncCounter();
		return true;

	}

	void Finish() {

		std::unique_lock<std::mutex> lock(*mtx);

		finish_processing = true;
		cv->notify_all();

		sync_wait->wait(lock, [&]() {
			return sync_counter == 0;
		});

		finish_processing = false;

	}

};