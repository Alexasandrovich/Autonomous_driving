#pragma once

#include <unordered_map>
#include <map>
#include <memory>



template<typename DataType>
struct CustomAllocator {
public:
	DataType* allocate(std::size_t newSize) {
		std::allocator<DataType> alloc;

		allocSize = newSize;

		DataType* pointer = alloc.allocate(allocSize);
		hashTable[pointer] = newSize;
		return pointer;
	}
	void deallocate(DataType* pointer) {
		std::allocator<DataType> alloc;

		auto it = hashTable.find(pointer);
		std::size_t allocatedSize = it->second;

		it = hashTable.erase(it);

		alloc.deallocate(pointer, allocatedSize);
	}

	void operator()(DataType* pointer) {
		deallocate(pointer);
	}

private:
	static inline std::unordered_map<DataType*, std::size_t> hashTable;
	std::size_t allocSize = 0;
};


template<typename DataType>
class ManagedDynamicArray {
public:
	ManagedDynamicArray(std::size_t size) {
		if (size > 0) {
			auto obj = InternalAllocType<CustomAllocator<DataType>>(alloc.allocate(size), CustomAllocator<DataType>());
			data = std::move(obj);
			reservedSize = size;
		}
		else {
			throw std::invalid_argument("Incorect input size in PtrManager constructor!");
		}
	}
	ManagedDynamicArray(const ManagedDynamicArray& other) {
		data = std::move(other.data);
		reservedSize = other.reservedSize;
	}

	void resetIfNeed(std::size_t newSize) {
		if (newSize > reservedSize) {
			data.reset(alloc.allocate(newSize));
			reservedSize = newSize;
		}
	}
	DataType* getPtr() {
		return data.get();
	}
	int getReservedSize() {
		return reservedSize;
	}
	void dataReset() {
		data.reset(nullptr);
	}

	ManagedDynamicArray& operator=(const ManagedDynamicArray& other) = delete;

	ManagedDynamicArray& operator=(const ManagedDynamicArray&& other) = delete;

private:
	CustomAllocator<DataType> alloc;

	template<typename Deleter>
	using InternalAllocType = std::unique_ptr < DataType[], Deleter >;
		
	InternalAllocType<CustomAllocator<DataType>> data;
	std::size_t reservedSize;
};


template<typename DataType>
class DataManager {
public:
	DataManager() : manager([](const ManagedExpandedPointer& obj1, const ManagedExpandedPointer& obj2) -> bool {
		if (obj1.second != obj2.second) {
			return obj1.second < obj2.second;
		}
		else {
			return obj1.first < obj2.first;
		}}) {}

	std::shared_ptr<ManagedDynamicArray<DataType>> getArray(std::size_t size) {
		timeUpdate();
		std::shared_ptr<ManagedDynamicArray<DataType>> object;
		for (auto& pair : manager) {
			if (pair.first.second >= size && pair.second.freeFlag) {
				pair.second.freeFlag = false;
				pair.second.timeLifeWhileFree = 0;
				pair.second.maxLifeTime += 1;
				return pair.first.first;
			}
		}
		if (!object) {
			object = std::make_shared<ManagedDynamicArray<DataType>>(size);
			manager[{object, size}] = { false, 0, 1 };
			return object;
		}
	}

	void timeUpdate() {
		for (auto iterator = manager.begin(); iterator != manager.end(); ) {
			if (iterator->second.freeFlag == true) {
				iterator->second.timeLifeWhileFree += 1;
				if (iterator->second.timeLifeWhileFree >= treshold || iterator->second.maxLifeTime >= maxLifeTime) {
					iterator = manager.erase(iterator);
					continue;
				}
			}
			else {
				++iterator;
			}
		}
	};

	bool releaseArray(const std::shared_ptr<ManagedDynamicArray<DataType>>& arrayPointer) {
		bool result = false;
		for (auto iterator = manager.begin(); iterator != manager.end(); ++iterator) {
			if (iterator->first.first == arrayPointer) {
				iterator->second.timeLifeWhileFree = 1;
				iterator->second.freeFlag = true;
				result = true;
				break;
			}
		}
		return result;
	}
	bool releaseArray(DataType* arrayPointer) {
		bool result = false;
		for (auto iterator = manager.begin(); iterator != manager.end(); ++iterator) {
			if (iterator->first.first.get()->getPtr() == arrayPointer) {
				iterator->second.timeLifeWhileFree = 1;
				iterator->second.freeFlag = true;
				result = true;
				break;
			}
		}
		return result;
	}

	void release() {
		for (auto iterator = manager.begin(); iterator != manager.end();) {
			iterator = manager.erase(iterator);
		}
	}

private:
	using ManagedExpandedPointer = std::pair<std::shared_ptr<ManagedDynamicArray<DataType>>, std::size_t>;
	class ObjectState {
	public:
		bool freeFlag;
		std::size_t timeLifeWhileFree;
		std::size_t maxLifeTime;
	};
	std::map<ManagedExpandedPointer, ObjectState, bool (*)(const ManagedExpandedPointer& obj1, const ManagedExpandedPointer& obj2)> manager;
	std::size_t treshold = 200;
	std::size_t maxLifeTime = 5;
};