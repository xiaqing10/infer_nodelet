#ifndef __MODULES_UNITS_SHMMEM_H__
#define __MODULES_UNITS_SHMMEM_H__

#include <stdint.h>
#include <string>
#include <memory>
#include <atomic>

#include "shm/futex.h"

namespace ehawkeye {
namespace modules  {
namespace units    {

constexpr int32_t shm_mem_magic  = 0x53484D31; // "SHM1"
const     int32_t shm_frame_size = 0x600000;   //6291456,bgr = 6M

typedef struct {
    int32_t  len;
    char     padding[4096 - sizeof(int32_t)];
    char     data[0];
} shmdata __attribute__((aligned(1)));

typedef struct {
    int32_t                magic;       // magic
    int32_t                version;     // 结构版本
    int32_t                count;       // 总数量
    std::atomic<uint64_t>  index;       // 读索引（字节偏移）
    std::atomic<int32_t>   mutex;       // 锁
    std::atomic<int32_t>   consumer;    // 消费者锁
} SharedHeader __attribute__((aligned(1)));

class shmmem {
public:
    using Ptr = std::shared_ptr<shmmem>;
    shmmem(const std::string name, int size = 30, bool create = false);
    ~shmmem();

    int write(void* data, int size);
    int write(void* header, int hsize, void* data, int dsize);
    int nocopyWriteKeep(void** data, int& size);
    int nocopyWriteRelease(int index, int size);
    int read(void* data, int size);
    int readByIndex(void* data, int size, int index);
    int nocopyRead(void** data, int& size);
    int nocopyRead(void** data, int& size, int index);

    std::string& path() { return this->name; }

private:
    bool initSharedRegion(bool create);
    bool initSyncPrimitives(bool initialize);

    int wait(std::atomic<int32_t>* sem);
    int post(std::atomic<int32_t>* sem);

    int offset()  { return (this->header->index.load(std::memory_order_relaxed) % this->header->count) * shm_frame_size; }
    int offsetByIndex(int index) { return (index % this->header->count) * shm_frame_size; }

private:
    int              fd;         // 描述符
    int              totalSize;  // 共享文件总大小
    std::string      name;       // 共享内存名字
    int32_t          elmSize;    // bgr图片张数
    SharedHeader*    header;     // 共享文件头
    void*            base;       // mmap映射起始位置
    char*            dataRegion; // 数据区域
    ehawkeye::modules::toolkit::condition_variable cond;
};

} // namespace units
} // namespace modules
} // namespace ehawkeye

#endif // __MODULES_UNITS_SHMMEM_H__