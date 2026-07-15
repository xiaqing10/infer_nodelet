/*************************************************************************
 * Copyright (C) [2020] by Cambricon, Inc.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO NOTIFIER SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *************************************************************************/

/************************************************************************
 *
 *  @file cndrv_api.h
 *
 *  @brief driver APIs provide programmable interfaces for users to develop
 *  their-owned programs, which includes device management, context
 *  management, memory management of both sides (devices and hosts), etc.
 *
 *  This header file contains programmable interface declarations,
 *      data types and error code definitions.
 *
 **************************************************************************/

#ifndef CNDRV_API_H_
#define CNDRV_API_H_

#define CNDRV_MAJOR_VERSION 0
#define CNDRV_MINOR_VERSION 6
#define CNDRV_PATCH_VERSION 106

#define CNDRV_VERSION (CNDRV_MAJOR_VERSION * 10000 + CNDRV_MINOR_VERSION * 100 + CNDRV_PATCH_VERSION)

/************************************************************************
 *  Include files
 ************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#if defined(WIN32) || defined(WINDOWS)
#else
    #include <sys/time.h>
#endif

#if defined(WIN32) || defined(WINDOWS)
#   if  defined(DRIVERAPI_IMPORTS)
#        define __CAMB_EXPORT __declspec(dllimport)
#else /*!DRIVERAPI_EXPORTS*/
#        define __CAMB_EXPORT __declspec(dllexport)
#endif /*DRIVERAPI_EXPORTS*/
#else /*!defined(WIN32) && !defined(WINDOWS)*/
#        define __CAMB_EXPORT
#endif /*defined(WIN32) || defined(WINDOWS)*/
#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* __cplusplus */


/**
 *  Invoke kernel extra type
 */
#define MLU_INVOKE_PARAM_BUFFER_POINTER ((void *)0x1UL)
#define MLU_INVOKE_PARAM_BUFFER_SIZE    ((void *)0x2UL)
#define MLU_INVOKE_PARAM_END            ((void *)0x0UL)

/**< Error codes */
typedef enum {
	MLU_SUCCESS = 0, /**< No error */
	MLU_ERROR_NOT_READY = 1, /*that asynchronous operations issued previously have not completed yet. This result is not actually an error*/
	/*error device manage*/
	MLU_ERROR_NOT_INITIALIZED = 3, /**< Failed to initialize */
	MLU_ERROR_DEINITIALIZED = 4, /**< Failed to deinitialize */
	MLU_ERROR_NO_DEVICE = 5, /**< No useful mlu device*/
	MLU_ERROR_INVALID_DEVICE = 6, /**< Invalid mlu device*/
	MLU_ERROR_RD_DRIVER_VERSION = 7, /*failed to read version*/
	MLU_ERROR_UNKNOWN_DEVICE = 8, /**< Unknown mlu device*/
	MLU_ERROR_INVALID_VALUE = 9, /**< Invalid value*/
	MLU_ERROR_INVALID_DEVICE_ATTRIBUTE = 10, /**< Invalid value*/

	/*error driver sbts*/
	MLU_SBTS_ERROR_SHARE_MEM_ALLOC = 21, /*Failed to alloc share memory*/
	MLU_SBTS_ERROR_FILL_TASK_DESC = 22, /*Failed to fill task descriptor*/
	MLU_SBTS_ERROR_IOCTL_PUT_TASK = 23, /*Ioctl failed to put task*/

	/*error queue*/
	MLU_QUEUE_ERROR_QUEUE_INVALID = 30, /*Queue legality chech error,queue not found*/

	/*error notifier*/
	MLU_NOTIFIER_ERROR_NOTIFIER_INVALID = 40, /*Notifier legality check error,notifier not found*/

	/*error cngdb*/
	MLU_KERNEL_DEBUG_ERROR_ACK = 50, /*ARM failed to write back data*/
	MLU_KERNEL_DEBUG_ERROR_TIMEOUT = 51, /*ARM write back data timeout*/

	/* error ops manage*/
	MLU_ERROR_INVALID_OPS = 101, /*Invalid operation*/
	MLU_OPS_ERROR_INVALID_VALUE = 102, /*Invalid value*/
	MLU_OPS_ERROR_OPEN = 103, /*Failed to open mlu device*/
	MLU_OPS_ERROR_CLOSE = 104, /*Failed to close mlu device*/
	MLU_OPS_ERROR_MAP = 105, /*A map or register operation has failed*/
	MLU_OPS_ERROR_UNMAP = 106, /*A unmap or unregister operation has failed*/
	MLU_OPS_ERROR_LSEEK = 107, /*LSEEK error*/
	MLU_OPS_ERROR_WRITE = 108, /*operation write error*/
	MLU_OPS_ERROR_OPERATING_SYSTEM = 109, /*operate system error*/
	MLU_OPS_ERROR_NOT_PERMITTED = 110, /*operation not permitted*/
	MLU_OPS_ERROR_NOT_SUPPORTED = 111,  /*operation not supported*/
	MLU_OPS_ERROR_READ_REGISTER = 120, /*read register error*/
	MLU_OPS_ERROR_WRITE_REGISTER = 121, /*write register error*/
	MLU_OPS_ERROR_READ_COREREGISTER = 122, /*read core register error*/
	MLU_OPS_ERROR_WRITE_COREREGISTER = 123, /*write core register error*/
	MLU_ERROR_OUT_OF_CORECOUNT = 124, /**< Invalid coreid*/
	MLU_ERROR_INIT_MUTEX = 125, /**< Fail to init mutex*/
	/* error memory manage*/
	MLU_ERROR_OUT_OF_MEMORY = 201, /**< Out of Memory*/
	MLU_MEMORY_ERROR_INVALID_VALUE = 202, /**< Invalid value*/
	MLU_MEMORY_ERROR_MLU_ADJSTACKMEM = 203, /**< Fail to adjust stack space memory*/
	MLU_ERROR_OUT_OF_PROCESS_MAX_MEMORY_USAGE = 204, /**< Out of the current max Memory usage*/
	MLU_MEMORY_ERROR_LEGALITY_CHECK = 205, /*failed to check legality when memory copy or free*/
	MLU_MEMORY_ERROR_MLU_MALLOC = 210, /**< Failed to mluMalloc*/
	MLU_MEMORY_ERROR_HOST_MALLOC = 211, /**< Failed to malloc*/
	MLU_MEMORY_ERROR_MLU_MALLOC_CONSTANT = 212, /**< Failed to mluMallocConstant*/
	MLU_MEMORY_ERROR_MLU_MALLOC_FRAMEBUFFER = 213, /**< Failed to mluMallocFrameBuffer*/
	MLU_MEMORY_ERROR_INVALID_ADDRESS = 214, /**< Invalid mlu address*/
	MLU_MEMORY_ERROR_MLU_FREE = 220, /**< Failed to mluFree*/
	MLU_MEMORY_ERROR_MLU_MEMMERGE = 225, /**< Failed to mluMemMerge*/
	MLU_MEMORY_ERROR_MERGE_OUT_OF_SIZE = 226, /*address to merge out of size*/
	MLU_MEMORY_ERROR_MERGE_DEVICE_BUSY = 227, /*An address is being merged or freed,cannot be copied*/
	MLU_MEMORY_ERROR_CPYHTOD = 230, /**< Failed to mluMemcpyHtoD*/
	MLU_MEMORY_ERROR_CPYHTOH = 231, /**< Failed to mluMemcpyHtoH*/
	MLU_MEMORY_ERROR_CPYDTOH = 240, /**< Failed to mluMemcpyDtoH*/
	MLU_MEMORY_ERROR_CPYDTOD = 241, /**< Failed to mluMemcpyDtoD*/
	MLU_MEMORY_ERROR_UNKNOWN_CACHE_OPS = 250, /**< Unknown cache operation*/
	MLU_MEMORY_ERROR_SMMU_OPEN = 251, /**< Failed to open smmu*/
	MLU_MEMORY_ERROR_SMMU_CLOSE = 252, /**< Failed to close smmu*/
	MLU_MEMORY_ERROR_GET_IOVA = 253, /**< Failed to get mlu vaddr*/
	MLU_MEMORY_ERROR_FREE_IOVA = 254, /**< Failed to free mlu vaddr*/
	MLU_MEMORY_ERROR_ION_OPEN = 255, /**< Failed to open ion*/
	MLU_MEMORY_ERROR_ION_CLOSE = 256, /**< Failed to close ion*/
	MLU_MEMORY_ERROR_CPYPTOP = 257, /**< Failed to Memcpy Peer to Peer*/
	MLU_MEMORY_ERROR_AccessPeer  = 258,
	MLU_MEMORY_ERROR_MLU_MEMSET = 259, /**< Failed to mluMemset*/
	MLU_ERROR_MALLOC_HOST = 260, /*failed to malloc host memory*/
	MLU_ERROR_FREE_HOST = 261, /*failed to free host memory*/
	MLU_MEMORY_ERROR_MANAGED = 262, /*failed to malloc MDR memory*/
	MLU_MEMORY_ERROR_MEMCHECK = 263, /*memory check error*/
	MLU_MEMORY_ERROR_EN_MEMCHECK = 264, /*memory check enable error*/
	MLU_MEMORY_ERROR_NOT_IN_DEVICE = 265, /*input address is not a MLU address*/
	MLU_ERROR_GET_HANDLE_HOST = 266, /* failed to get host memory handle */
	MLU_ERROR_OPEN_HANDLE_HOST = 267, /* failed to open host memory handle */
	MLU_ERROR_CLOSE_HANDLE_HOST = 268, /* failed to close host memory handle */
	MLU_ERROR_GET_HOST_MEM_RANGE = 269, /* failed to close host memory handle */

	/*error task QUEUE manage*/
	MLU_TASK_ERROR_INVALID_VALUE = 305, /**< Invalid value*/
	MLU_TASK_ERROR_QUEUE = 306, /**< Failed to push task*/
	MLU_TASK_ERROR_SEC_CANCEL = 307, /**< Failed to push(in security os)*/
	MLU_TASK_ERROR_OUT_OF_TIME = 308, /**< Task too much time*/
	MLU_TASK_ERROR_UNKNOW = 350, /**< Unknown error*/
	/* error queue manage*/
	MLU_QUEUE_ERROR_CREATE = 401, /*failed to create queue*/
	MLU_QUEUE_ERROR_CREATE_WithPriority = 402, /*failed to create queue with priority*/

	MLU_QUEUE_ERROR_DESTROY = 403, /*failed to destroy queue*/
	MLU_QUEUE_ERROR_QUERY = 404, /*failed to query queue*/
	MLU_QUEUE_ERROR_SYNC = 405, /*failed to sync queue*/
	MLU_QUEUE_ERROR_WAIT_NOTIFIER = 406,/*failed to do queue-wait-notifier*/

	MLU_QUEUE_ERROR_OUT_OF_MEMORY = 407, /*out of memory*/
	MLU_QUEUE_ERROR_GBC = 408, /*gbc error*/
	MLU_QUEUE_ERROR_INSTRUCTION_INVALID = 409, /*instruction invalid*/
	MLU_QUEUE_ERROR_LOCAL_MEMORY_SIZE = 410, /*local memory size error*/
	MLU_QUEUE_ERROR_NO_RESOURCE = 411, /*no resource*/

	MLU_QUEUE_ERROR_DMA_MISALIGNED = 412,/*c10 only*/
	MLU_QUEUE_ERROR_OUT_OF_CHIP_MEMORY = 413,/*c10 only*/
	MLU_QUEUE_ERROR_BARRIER_TIMEOUT = 420, /*barrier timeout*/
	MLU_QUEUE_ERROR_INSTRUCTION_CONFIG = 421, /*instruction config error*/
	MLU_QUEUE_ERROR_IPU_UNKNOWN = 422, /*ipu unknown error*/
	MLU_QUEUE_ERROR_INVALID_QUEUE = 423, /*invalid queue*/

	MLU_ERROR_INSTRUCTION_CONFIG_ERROR = 430, /*IPU error instruction config error*/
	MLU_ERROR_SCALAR_CALCULATE_ERROR = 431, /*IPU scalar calculate error*/
	MLU_ERROR_INSTRUNRAMION_INVALID = 432, /*IPU instruction invalid*/
	MLU_ERROR_INSTRUNRAMION_ISSUE_TIMEOUT = 433, /*IPU instruction timeout*/
	MLU_ERROR_READ_SRAM_OVERFLOW = 434, /*IPU read sram overflow*/
	MLU_ERROR_READ_DRAM_OVERFLOW = 435, /*IPU read dram overflow*/
	MLU_ERROR_READ_NRAM_OVERFLOW = 436, /*IPU read nram overflow*/
	MLU_ERROR_READ_WRAM_OVERFLOW = 437, /*IPU read wram overflow*/
	MLU_ERROR_READ_ADDRESS_ERROR = 438, /*IPU read address error*/
	MLU_ERROR_WRITE_SRAM_OVERFLOW = 439, /*IPU write sram overflow*/
	MLU_ERROR_WRITE_DRAM_OVERFLOW = 440, /*IPU write dram overflow*/
	MLU_ERROR_WRITE_NRAM_OVERFLOW = 441, /*IPU write nram overflow*/
	MLU_ERROR_WRITE_WRAM_OVERFLOW = 442, /*IPU write wram overflow*/
	MLU_ERROR_WRITE_ADDRESS_ERROR = 443, /*IPC write address error*/
	MLU_ERROR_DECOMPRESS_ERROR = 444, /*IPU decompress error*/
	MLU_ERROR_COMPRESS_ERROR = 445, /*IPU compress error*/
	MLU_ERROR_SRAM_ECC_ERROR = 446, /*IPU sram ecc error*/
	MLU_ERROR_DRAM_ECC_ERROR = 447, /*IPU dram ecc error*/
	MLU_ERROR_NRAM_ECC_ERROR = 448, /*IPU nram ecc error*/
	MLU_ERROR_WRAM_ECC_ERROR = 449, /*IPU wram ecc error*/
	MLU_ERROR_READ_SRAM_TIMEOUT = 450, /*read sram timeout*/
	MLU_ERROR_READ_DRAM_TIMEOUT = 451, /*read dram timeout*/
	MLU_ERROR_READ_NRAM_TIMEOUT = 452, /*read nram timeout*/
	MLU_ERROR_READ_WRAM_TIMEOUT = 453, /*read wram timeout*/
	MLU_ERROR_WRITE_SRAM_TIMEOUT = 454, /*write sram timeout*/
	MLU_ERROR_WRITE_DRAM_TIMEOUT = 455, /*write dram timeout*/
	MLU_ERROR_WRITE_NRAM_TIMEOUT = 456, /*write nram timeout*/
	MLU_ERROR_WRITE_WRAM_TIMEOUT = 457, /*write wram timeout*/
	MLU_ERROR_READ_SRAM_RESP = 458, /*respond error reading sram*/
	MLU_ERROR_READ_DRAM_RESP = 459, /*respond errpr reading dram*/
	MLU_ERROR_READ_NRAM_RESP = 460, /*respond error reading nram*/
	MLU_ERROR_READ_WRAM_RESP = 461, /*respond error reading wram*/
	MLU_ERROR_WRITE_SRAM_RESP = 462, /*respond error writing sram*/
	MLU_ERROR_WRITE_DRAM_RESP = 463, /*respond error writing dram*/
	MLU_ERROR_WRITE_NRAM_RESP = 464, /*respond error writing nram*/
	MLU_ERROR_WRITE_WRAM_RESP = 465, /*respond error writing wram*/
	MLU_ERROR_BUS = 466, /*bus error*/
	MLU_ERROR_READ_NRAM_COM_OVERFLOW = 467, /*read CT com overflow*/
	MLU_ERROR_READ_NRAM_COM_ERROR = 468, /*read CT com error*/
	MLU_ERROR_SCALAR_NRAM_DATA_ERROR = 469, /*scalar error*/
	MLU_ERROR_WRITE_NRAM_DATA_OVERFLOW = 470, /*write nram data overflow*/
	MLU_ERROR_WRITE_NRAM_DATA_ERROR = 471, /*write nram data error*/
	MLU_ERROR_READ_WRAM_ERROR = 472, /*read wram error*/
	MLU_ERROR_WRITE_NRAM_DATA_TIMEOUT = 473, /*write nram timeout*/
	MLU_ERROR_READ_NRAM_DATA_TIMEOUT = 474, /*read nram timeout*/
	MLU_ERROR_NFU_ERROR = 475, /*CT NFU error*/
	MLU_ERROR_BARRIER_TIMEOUT = 476, /*barrier timeout*/
	MLU_ERROR_PV_TIMEOUT = 477, /*PV timeout*/
	MLU_ERROR_ICACHE_ECC_ERROR = 478, /*Icache ecc error*/
	MLU_ERROR_WATCH_DOG_TIMEOUT = 479, /*watchdog timeout*/

	/* dma async task error */
	MLU_ERROR_DMA_ASYNC_ERR = 480,

	/* error notifier manage*/
	MLU_NOTIFIER_ERROR_CREATE = 501, /*create notifier error*/
	MLU_NOTIFIER_ERROR_DESTROY = 502, /*destroy notifier error*/
	MLU_NOTIFIER_ERROR_WAIT = 503, /*notifier wait error*/
	MLU_NOTIFIER_ERROR_QUERY = 504, /*query notifier error*/
	MLU_NOTIFIER_ERROR_PLACE_NOTIFY = 505, /*place nofify error*/
	MLU_NOTIFIER_ERROR_ELAPSED_TIME = 506, /*get elapetime error*/
	MLU_NOTIFIER_ERROR_NO_RESOURCE = 507, /*notifier no resource*/

	/* atomic manager*/
	MLU_ATOMIC_ERROR_PLACE_QUEUE_TASK = 550, /* place atomic task error*/
	MLU_ATOMIC_ERROR_PLACE_TASK = 551, /* do atomic task error*/

	/* error kernel manage*/
	MLU_ERROR_INVOKE = 607, /*error invoke*/

	/* visible_devices function increasing*/
	MLU_ERROR_ENV_MLU_VISIBLE_DEVICES = 608,/*get visible device error*/

	MLU_CNGDB_ERROR = 701, /*cngdb error*/
	MLU_INVALID_INFO_TYPE = 702, /*invalid info type*/
	MLU_CNGDB_CORE_ID_WRONG = 703, /*cngdb core ID error*/

	MLU_ERROR_IPC_GET_MEM_HANDLE = 801, /*ipc get handle error*/
	MLU_ERROR_IPC_OPEN_MEM_HANDLE = 802, /*ipc open handle error*/
	MLU_ERROR_IPC_CLOSE_MEM_HANDLE = 803, /*ipc close handle error*/
	MLU_ERROR_IPC_GET_NOTIFIER_HANDLE = 804, /*ipc-event get handle error*/
	MLU_ERROR_IPC_OPEN_NOTIFIER_HANDLE = 805, /*ipc-event open handle error*/

	/* error monitor */
	MLU_ERROR_GET_MEMINFO_ERROR = 950,
	MLU_ERROR_GET_CARDINFO_ERROR = 951,
	MLU_ERROR_GET_ECCINFO_ERROR = 952,
	MLU_ERROR_GET_POWERINFO_ERROR = 953,
	MLU_ERROR_GET_HEALTHSTATEINFO_ERROR = 954,
	MLU_ERROR_GET_IPUUTILINFO_ERROR = 955,
	MLU_ERROR_GET_FREQINFO_ERROR = 956,

	/* ncs return code */
	MLU_ERROR_NCS_START = 1000,
	MLU_ERROR_NCS_PORT_NOT_IN_RANGE = 1001,
	MLU_ERROR_NCS_PORT_LINK_ERR = 1002,
	MLU_ERROR_NCS_QP_CREATE_PARAM_INVALID = 1003,
	MLU_ERROR_NCS_QP_CREATE_NO_RESOURCE = 1004,
	MLU_ERROR_NCS_QP_MODIFY_PAIRING = 1005,
	MLU_ERROR_NCS_QP_MODIFY_LQP_INVALID = 1006,
	MLU_ERROR_NCS_QP_MODIFY_RQP_INVALID = 1007,
	MLU_ERROR_NCS_QP_MODIFY_NO_RESOURCE = 1008,
	MLU_ERROR_NCS_QP_DESTROY_PARAM_INVALID = 1009,
	MLU_ERROR_NCS_QP_DESTROY_NOT_EXISTS = 1010,
	MLU_ERROR_NCS_QP_DESTROY_BUSY = 1011,
	MLU_ERROR_NCS_QP_DESTROY_USER_INVALID = 1012,
	MLU_ERROR_NCS_TEMPLATE_CREATE_PARAM_INVALID = 1013,
	MLU_ERROR_NCS_TEMPLATE_CREATE_NO_RESOURCE = 1014,
	MLU_ERROR_NCS_TEMPLATE_CREATE_LOAD_ERR = 1015,
	MLU_ERROR_NCS_TEMPLATE_DESTROY_PARAM_INVALID = 1016,
	MLU_ERROR_NCS_TEMPLATE_DESTROY_NOT_EXISTS = 1017,
	MLU_ERROR_NCS_TEMPLATE_DESTROY_BUSY = 1018,
	MLU_ERROR_NCS_TEMPLATE_DESTROY_USER_INVALID = 1019,
	MLU_ERROR_NCS_PORT_OB_DISABLED = 1020,
	MLU_ERROR_NCS_PORT_IB_DISABLED = 1021,
	MLU_ERROR_NCS_PORT_DISABLED = 1022,
	MLU_ERROR_NCS_UNSUPPORT = 1023,

	/* ncs error code of execution */
	MLU_ERROR_NCS_UPDATA_TOPOLOGY = 1200,
	MLU_ERROR_NCS_LAUNCH_KERNEL = 1201,
	MLU_ERROR_NCS_GET_PORT_INFO = 1202,
	MLU_ERROR_NCS_GET_DEVICE_UUID = 1203,
	MLU_ERROR_NCS_PORT_ID_INVALID = 1204,
	MLU_ERROR_NCS_DESC_INVALID = 1205,
	MLU_ERROR_NCS_ERROR = 1298,
	MLU_ERROR_NCS_END = 1299,

	/* error code of module */
	MLU_ERROR_INVALID_FATBIN = 1300,
	MLU_ERROR_NOT_SUPPORT = 1301,
	MLU_ERROR_INVALID_MODULE = 1302,
	MLU_ERROR_INVALID_KERNEL_NAME = 1303,
	MLU_ERROR_INVALID_KERNEL = 1304,
	MLU_ERROR_FATBIN_RET_VERSION_MISMATCH = 1305,
	MLU_ERROR_FATBIN_RET_FAILED_PARSE_KERNEL_INFO = 1306,
	MLU_ERROR_FATBIN_RET_FAILED_LOAD_KERNEL = 1307,
	MLU_ERROR_FATBIN_RET_FAILED_MALLOC = 1308,
	MLU_ERROR_FATBIN_RET_FAILED_MEMSET = 1309,
	MLU_ERROR_FATBIN_RET_FAILED_MEMCPY = 1310,
	MLU_ERROR_FATBIN_RET_FAILED_INVALID = 1311,
	MLU_ERROR_FATBIN_RET_FAILED_NO_KERNEL = 1312,
	MLU_ERROR_FATBIN_RET_RELOCATION_ERROR = 1313,

	MLU_ERROR_UNKNOWN = 2000 /*error unknown*/
	/* end*/
} MLUresult;

typedef uint64_t MLUdev;
typedef void *MLUqueue;
typedef void *MLUaddr;
typedef void *MLUnotifier;
typedef void *HOSTaddr;
typedef void *MLUIpcMemHandle;
typedef void *MLUIpcNotifierHandle;

/**
 * Device properties
 */
typedef enum MLUdevice_attribute_enum {
	MLU_DEVICE_ATTRIBUTE_CLUSTER_COUNT = 0,                      /**< Maximum cluster of the device */
	MLU_DEVICE_ATTRIBUTE_MAX_MCORES_PER_CLUSTER = 1,             /**< Maximum MLU core of the per cluster */
	MLU_DEVICE_ATTRIBUTE_MCORE_TYPE = 2,			     /**< the type of the mlu core */
	MLU_DEVICE_ATTRIBUTE_ECC_ENABLED = 3,                       /**< Device has ECC support enabled */
	MLU_DEVICE_ATTRIBUTE_MEMORY_CLOCK_RATE = 4,                 /**< Maximum Peak memory clock frequency in kilohertz */
	MLU_DEVICE_ATTRIBUTE_CLUSTER_CLOCK_RATE = 5,                /**< Maximum Cluster clock frequency in kilohertz */
	MLU_DEVICE_ATTRIBUTE_PCI_BUS_ID = 6,                        /**< PCI bus ID of the device */
	MLU_DEVICE_ATTRIBUTE_PCI_DEVICE_ID = 7,                     /**< PCI device ID of the device */
	MLU_DEVICE_ATTRIBUTE_PCI_DOMAIN_ID = 8,                     /**< PCI domain ID of the device */
	MLU_DEVICE_ATTRIBUTE_STACK_SIZE_PER_MCORE = 9,               /**< Maximum stack memory available per MLU Core in MB */
	MLU_DEVICE_ATTRIBUTE_SRAM_SIZE_PER_MCORE = 10,               /**< Maximum sram memory available per MLU Core in bytes */
	MLU_DEVICE_ATTRIBUTE_TOTAL_MEMORY_SIZE = 11,               /**< Maximum available total memory in MB */
	MLU_DEVICE_ATTRIBUTE_GLOBAL_MEMORY_BUS_WIDTH = 12,           /**< Global memory bus width in bits */
	MLU_DEVICE_ATTRIBUTE_SYSTEM_CACHE_SIZE = 13,                     /**< Size of SYSTEM cache in bytes */
	MLU_DEVICE_ATTRIBUTE_MAX_DIM_X = 14,                          /**< Maximum block dimension X */
	MLU_DEVICE_ATTRIBUTE_MAX_DIM_Y = 15,                          /**< Maximum block dimension Y */
	MLU_DEVICE_ATTRIBUTE_MAX_DIM_Z = 16,                          /**< Maximum block dimension Z */
	MLU_DEVICE_ATTRIBUTE_MAX_QUEUE = 17,                         /**< Maximum queue count */
	MLU_DEVICE_ATTRIBUTE_MAX_NOTIFIER = 18,                         /**< Maximum notifier count */
	MLU_DEVICE_ATTRIBUTE_KERNEL_EXEC_LIMIT = 19,                 /**< Specifies whether there is a run time limit on kernels */
	MLU_DEVICE_ATTRIBUTE_QUEUE_PRIORITIES_SUPPORTED = 20,       /**< Device supports queue priorities */
	MLU_DEVICE_ATTRIBUTE_PORT_COUNT = 21,	                     /**< Maximum port of the device */

	/*Add new to here!*/
	MLU_DEVICE_ATTRIBUTE_MAX
} MLUdevice_attribute;

/*Thread blocking spin wait, this mode has high CPU usage*/
#define CN_QUEUE_SYNC_BLOCK  0
/*Thread is blocked and waiting for sleep, this mode has low CPU usage*/
#define CN_QUEUE_SYNC_WAIT   1

#define BOARD_MODEL_NAME_LEN 32

/**
 * Memory info struct
 */
typedef struct MLUMemoryInfo{
	unsigned int version;
	uint64_t phy_total;             /**< MLU physical total memory, unit:MB*/
	uint64_t phy_used;              /**< MLU physical used memory, unit:MB*/
	uint64_t chl_num;               /**< Memory channel number, indicate the num of chl count*/
	uint64_t *each_chl;             /**< Memory used each channel, unit:MB*/
} MLUMemoryInfo_t;

/**
 * Power info struct
 */
typedef struct MLUPowerInfo {
	unsigned int version;
	/**< board max power unit:W*/
	uint16_t max_power;
	/**< current power unit:W*/
	uint16_t power_usage;
	/**< MLU fan speed，the percentage of the max fan speed*/
	uint16_t fan_speed;
	/**< user set this var to kernel to indicate userspace length of (*temp) */
	uint8_t temperature_num;
	/**< temperature variable, (type: int8_t) */
	int8_t *temperature;

	uint16_t power_usage_decimal;
} MLUPowerInfo_t;

/**
 * Health State info struct
 */
typedef struct MLUHealthStateInfo {
	unsigned int version;

	uint8_t host_state;
	uint8_t card_state;
}MLUHealthStateInfo_t;

/**
 * Health State info struct
 */
typedef struct MLUECCInfo {
	unsigned int version;

	/**< single single-bit error / corrected*/
	uint64_t single_biterr;
	/**< multiple single-bit error / corrected*/
	/* when 290 it means two bit err */
	uint64_t multi_biterr;
	/**< single multiple-bits error / uncorrected*/
	uint64_t single_multierr;
	/**< multiple multiple-bits error / uncorrected*/
	uint64_t multi_multierr;
	/**< corrected error*/
	uint64_t corrected_err;
	/**< uncorrected error*/
	uint64_t uncorrect_err;
	/**< ECC error total times*/
	uint64_t total_err;
}MLUECCInfo_t;

/**
 * IPU UTIL info struct
 */
typedef struct MLUIPUUtilInfo {
	unsigned int version;

	uint16_t chip_util;
	uint8_t core_num;
	uint8_t *core_util;
} MLUIPUUtilInfo_t;

/**
 * freq info struct
 */
typedef struct MLUFreqInfo {
	unsigned int version;

	/*MHz*/
	uint32_t ipu_freq;
	uint32_t ddr_freq;
}MLUFreqInfo_t;


typedef struct MLUCardInfo {
	unsigned int version;

	uint16_t mcu_major_ver; /**< MCU major id*/
	uint16_t mcu_minor_ver; /**< MCU minor id*/
	uint16_t mcu_build_ver; /**< MCU build id*/
	uint16_t driver_major_ver; /**< Driver major id*/
	uint16_t driver_minor_ver; /**< Driver minor id*/
	uint16_t driver_build_ver; /**< Driver build id*/

	uint32_t subsystem_id; /**PCIe Sub-System ID**/
	uint32_t device_id; /**PCIe Device ID**/
	uint16_t vendor_id; // NOLINT /**PCIe Vendor ID**/

	uint16_t ipu_cluster; /** card cluster count*/
	uint32_t ipu_core; /** card core count*/

	uint32_t max_speed; /**PCI max speed**/
	uint32_t max_width; /**PCI width**/

	uint64_t card_sn; /** card SN in hex*/

	char board_model[BOARD_MODEL_NAME_LEN]; /**boaed model name*/

} MLUCardInfo_t;


typedef enum MLUAtomicOpType {
	MLU_ATOMIC_OP_REQUEST,
	MLU_ATOMIC_OP_COMPARE,
} MLUAtomicOpType_t;

enum MLUAtomicCompFlag {
	MLU_ATOMIC_COMPARE_EQUAL = 0,   /* op_data1  = *op_addr */
	MLU_ATOMIC_COMPARE_LESS_EQUAL,  /* op_data1 <= *op_addr */
	MLU_ATOMIC_COMPARE_LESS,        /* op_data1 <  *op_addr */
};

enum MLUAtomicReqFlag {
	MLU_ATOMIC_REQUEST_DEFAULT = 0,
	MLU_ATOMIC_REQUEST_ADD = MLU_ATOMIC_REQUEST_DEFAULT ,  /* atomic add */
	MLU_ATOMIC_REQUEST_SET,            /* set val */
	MLU_ATOMIC_REQUEST_CLEAR,          /* reset ptr */
};

/*************************************************************************
 * Initialization and Exit
  *************************************************************************/

/**
 * @brief Initialize driver environment in current process space.
 *
 * Initializes this API must be called before any other driver API calls.
 *
 * @param  Flags[in] reserved for further use, pass 0 as well.
 * @return MLU_SUCCESS if success, otherwise with the error code.
 */
__CAMB_EXPORT extern MLUresult mluInit(unsigned int Flags);

/**
 * @brief Exit driver environment in current process space.
 *
 * This API should be called after any other driver API calls
 * @param Flags[in] reserved for further use, pass 0 as well.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluExit(unsigned int Flags);

/******************************************************************************
 * Version
  ******************************************************************************/

/**
 * @brief Get the mlu device's driver version.
 *
 * @param dev[in] device descriptor.
 * @param driverVersion[out] a pointer to pointer for retrieving driver version.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDriverGetVersion(MLUdev dev, int *driverVersion);

/******************************************************************************
 * Device management
  ******************************************************************************/
/**
 * @brief Get the string of an error code.
 *
 * @param count[out] a pointer to pointer for retrieving the count of devices.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluGetErrorName(MLUresult error, const char **pStr);
/**
 * @brief Get the string description of an error code.
 *
 * @param count[out] a pointer to pointer for retrieving the count of devices.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluGetErrorString(MLUresult error, const char **pStr);
/**
 * @brief Get the count of MLU devices in the system.
 *
 * @param count[out] a pointer to pointer for retrieving the count of devices.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluGetDeviceCount(int *count);

/**
 * @brief Get device descriptor by a given device ordinal.
 *
 * @param dev[out] a pointer to pointer for retrieving device descriptor.
 * @param ordinal[in] the device ordinal to get device descriptor.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDeviceGet(MLUdev *dev, int ordinal);

/**
 * @brief Returns an identifer string for the device
 *
 * Returns an ASCII string identifying the device in the NULL-terminated
 * string pointed to by name. len specifies the maximum length of the
 * string that may be returned.
 *
 * @param name - Returned identifier string for the device
 * @param len  - Maximum length of string to store in \p name
 * @param dev  - Device to get identifier string for
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDeviceGetName(char *name, unsigned int len, MLUdev dev);

/**
 * @brief Returns information about the device.
 *
 * @param pi[out] a pointer to pointer for retrieving device attribute value.
 * @param attrib[in] Device attribute to query.
 * @param dev[in] device descriptor.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDeviceGetAttribute(int *pi, MLUdevice_attribute attrib, MLUdev dev);

/**
 * @brief Get the total mlu memory size in byte.
 *
 * @param dev[in] device descriptor.
 * @param bytes[out] a pointer to retrieving the memory amount on mlu device.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDeviceTotalMem(MLUdev dev, uint64_t *bytes);

/**
 * @brief Open the mlu device.
 *
 * @param dev[in] device descriptor.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDeviceOpen(MLUdev dev);

/**
 * @brief Close the mlu device.
 *
 * @param dev[in] device descriptor.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDeviceClose(MLUdev dev);

/******************************************************************************
 * Memory management
  ******************************************************************************/

/**
 * @brief Allocate mlu memory with given size.
 *
 * @param mlu_addr[out] a pointer to pointer for retrieving allocated mlu memory.
 * @param dev[in] device descriptor.
 * @param size[in] the number of bytes of allocated mlu memory.
 * @param alignment[in] required alignment of the allocation.
 * @param channel[in] the channel id of the allocated mlu memory.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMalloc(MLUaddr *mlu_addr, MLUdev dev, uint64_t size,
	uint64_t alignment, unsigned int channel);
/**
 * @brief Set mlu memory with given value and size.
 *
 * @param mlu_addr[in] required destination mlu address.
 * @param dev[in] device descriptor.
 * @param value[in] value to set for each byte of specified mlu memory.
 * @param size[in] the number of bytes of specified mlu memory.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMemset(MLUaddr mlu_addr, MLUdev dev, int value, uint64_t size);
/**
 * @brief Allocate mlu memory with given size and set 0.
 *
 * @param mlu_addr[out] a pointer to pointer for retrieving allocated mlu memory.
 * @param dev[in] device descriptor.
 * @param size[in] the number of bytes of allocated mlu memory.
 * @param alignment[in] required alignment of the allocation.
 * @param channel[in] the channel id of the allocated mlu memory.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluZmalloc(MLUaddr *mlu_addr, MLUdev dev,
	uint64_t size, uint64_t alignment, unsigned int channel);
/**
 * @brief Allocate mlu constant memory with given size.
 *
 * @param mlu_addr[out] a pointer to pointer for retrieving allocated mlu constant memory.
 * @param dev[in] device descriptor.
 * @param size[in] the number of bytes of allocated mlu constant memory.
 * @param alignment[in] required alignment of the allocation.
 * @param channel[in] the channel id of the allocated mlu constant memory.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMallocConstant(MLUaddr *mlu_addr, MLUdev dev,
	uint64_t size, uint64_t alignment, unsigned int channel);

/**
 * @brief Allocate FrameBuffer with given size that is mlu contiguous physical memory .
 *
 * @param mlu_addr[out] a pointer to pointer for retrieving allocated FrameBuffer.
 * @param dev[in] device descriptor.
 * @param size[in] the number of bytes of allocated FrameBuffer.
 * @param alignment[in] required alignment of the allocation.
 * @param channel[in] the channel id of the allocated FrameBuffer.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMallocFrameBuffer(MLUaddr *mlu_addr, MLUdev dev,
	uint64_t size, uint64_t alignment, unsigned int channel);
/**
 * @brief merge several mlu memory to a mlu memory.
 *
 * notice: after merging several mlu memory to a mlu memory, the addr meta of
   several mlu memory will be invalid.
 *
 * @param mluMerge_addr[out] a pointer to pointer for retrieving merge memory.
 * @param dev[in] device descriptor.
 * @param count[in] the count of mlu memory that need to be merged.
 * @param mlu_addrs[in] the addr meta of mlu memory that need to be merged.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMemMerge(MLUaddr *mluMerge_addr, MLUdev dev, unsigned int count, const MLUaddr *mlu_addrs);

/**
 * @brief Free the mlu device.
 *
 * @param dev[in] device descriptor.
 * @param mlu_addr[in] required free mlu address.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluFree(MLUdev dev, MLUaddr mlu_addr);

/**
 * @brief Copy nBytes from src host address to dst mlu address.
 *
 * @param dev[in] device descriptor.
 * @param mlu_addr[in] required destination mlu address.
 * @param host_addr[in] required source host address.
 * @param size[in] the number of bytes to be copied.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 * Note: The user should ensure that the size is less than the memory size
 *       they applied for with general "mluMalloc" or "mluMallocHost" API
 * Note: To maintain more stable performance, try using 4 bytes aligned address
 */
__CAMB_EXPORT extern MLUresult mluMemcpyHtoD(MLUdev dev, MLUaddr mlu_addr, const HOSTaddr host_addr, uint64_t size);

/**
 * @brief Copy nBytes from src mlu address to dst host address.
 *
 * @param dev[in] device descriptor.
 * @param mlu_addr[in] required source mlu address.
 * @param host_addr[in] required destination host address.
 * @param size[in] the number of bytes to be copied.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 * Note: The user should ensure that the size is less than the memory size
 *       they applied for with general "mluMalloc" or "mluMallocHost" API
 * Note: To maintain more stable performance, try using 4 bytes aligned address
 */
__CAMB_EXPORT extern MLUresult mluMemcpyDtoH(MLUdev dev, const MLUaddr mlu_addr, HOSTaddr host_addr, uint64_t size);

/**
 * @brief Copy nBytes from src mlu address to dst mlu address.
 *
 * @param dev[in] device descriptor.
 * @param src_addr[in] required source mlu address.
 * @param dst_addr[in] required destination mlu address.
 * @param size[in] the number of bytes to be copied.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMemcpyDtoD(MLUdev dev, MLUaddr src_addr, MLUaddr dst_addr, uint64_t size);

/**
 * @brief Copy nBytes from src mlu address to dst mlu address asynchronously.
 *
 * @param dev[in] device descriptor.
 * @param src_addr[in] required source mlu address.
 * @param dst_addr[in] required destination mlu address.
 * @param size[in] the number of bytes to be copied.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMemcpyDtoDAsync(MLUdev dev, MLUaddr src_addr, MLUaddr dst_addr, uint64_t size, MLUqueue hqueue);

/**
 * @brief Copy nBytes from src host address to dst mlu address asynchronously.
 *
 * @param dev[in] device descriptor.
 * @param mlu_addr[in] required destination mlu address.
 * @param host_addr[in] required source host address.
 * @param size[in] the number of bytes to be copied.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 * Note: The user should ensure that the size is less than the memory size
 *       they applied for with general "mluMalloc" or "mluMallocHost" API
 * Note: To maintain more stable performance, try using 4 bytes aligned address
 */
__CAMB_EXPORT extern MLUresult mluMemcpyHtoDAsync(MLUdev dev, MLUaddr mlu_addr, const HOSTaddr host_addr, uint64_t size, MLUqueue hqueue);

/**
 * @brief Copy nBytes from src mlu address to dst host address asynchronously.
 *
 * @param dev[in] device descriptor.
 * @param mlu_addr[in] required destination mlu address.
 * @param host_addr[in] required source host address.
 * @param size[in] the number of bytes to be copied.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 * Note: The user should ensure that the size is less than the memory size
 *       they applied for with general "mluMalloc" or "mluMallocHost" API
 * Note: To maintain more stable performance, try using 4 bytes aligned address
 */
__CAMB_EXPORT extern MLUresult mluMemcpyDtoHAsync(MLUdev dev, MLUaddr mlu_addr, const HOSTaddr host_addr, uint64_t size, MLUqueue hqueue);


/**
 * @brief Copy nBytes devices memory between two mlu device asynchronously.
 *
 * @param dev[in] device descriptor.
 * @param dst_addr[in] required destination mlu address.
 * @param src_addr[in] required source mlu address.
 * @param size[in] the number of bytes to be copied.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMemcpyPeerAsync(MLUdev dev, MLUaddr dst_addr, MLUaddr src_addr, uint64_t size, MLUqueue hqueue);

/**
 * @brief Copy nBytes devices memory between two mlu device.
 *
 * @param dev[in] device descriptor.
 * @param dst_addr[in] required destination mlu address.
 * @param src_addr[in] required source mlu address.
 * @param size[in] the number of bytes to be copied.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMemcpyPeer2Peer(MLUdev dev, MLUaddr dst_addr, MLUaddr src_addr, uint64_t size);

/**
 * @brief Asynchronously Copy nBytes from src peer c20 address to dst peer c20 address.
 *
 * @param pdesc[in] a pointer to source device descriptor.
 * @param dst_addr[in] required destination c20 address.
 * @param src_addr[in] required source c20 address.
 * @param hqueue[in] queue handle created by calling c20CreateQueue or c20CreateQueueWithPriority.
 * @param size[in] the number of bytes to be copied.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */

__CAMB_EXPORT extern MLUresult mluMemcpyPeer2PeerAsync(MLUdev dev, MLUaddr dst_addr, MLUaddr src_addr, MLUqueue hqueue, uint64_t size);

/**
 * @brief check the peer2peer ability .
 *
 * @param devSrc[in] src device descriptor.
 * @param devDst[in] dst device descriptor.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluMemcpyPeerAble(MLUdev devSrc, MLUdev devDst);

/******************************************************************************
 * Queue management
  ******************************************************************************/

/**
 * @brief Get the Maximum count of queue in the device.
 *
 * @param dev[in] device descriptor.
 * @param count[out] a pointer to pointer for retrieving the Maximum count of queue.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluGetMaxQueueCount(MLUdev dev, unsigned int *count);

/**
 * @brief Create a new queue after calling this function.
 *
 * @param dev[in] device descriptor.
 * @param phqueue[out] a pointer to pointer for retrieving the new created Queue handle.
 * @param  flags[in] indicates the way in which the queue will synchronise.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluCreateQueue(MLUdev dev, MLUqueue *phqueue, int flags);

/**
 * @brief Create a new queue with priority after calling this function.
 *
 * @param dev[in] device descriptor.
 * @param phqueue[out] a pointer to pointer for retrieving the new created Queue handle.
 * @param  flags[in] indicates the way in which the queue will synchronise.
 * @param  priority[in] the priority of the created queue.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluCreateQueueWithPriority(MLUdev dev, MLUqueue *phqueue, unsigned int flags, int priority);

/**
 * @brief Destroy a queue after calling this function.
 *
 * @param dev[in] device descriptor.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDestroyQueue(MLUdev dev, MLUqueue hqueue);

/**
 * @brief Function to query queue status on the completion of all precedent tasks.
 *
 * @param dev[in] device descriptor.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 * Note : function returns 1 means that the queue has unfinished tasks.
 */
__CAMB_EXPORT extern MLUresult mluQueryQueue(MLUdev dev, MLUqueue hqueue);

/**
 * @brief Function should be blocked until all precedent tasks in the queue are completed.
 *
 * @param dev[in] device descriptor.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluQueueSync(MLUdev dev, MLUqueue hqueue);

/**
 * @brief Make a queue wait on an notifier.
 *
 * @param dev[in] device descriptor.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @param hnotifier[in] Notifier to wait on(may not be NULL).
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluQueueWaitNotifier(MLUdev dev, MLUqueue hqueue, MLUnotifier hnotifier);

/******************************************************************************
 * Notifier management
  ******************************************************************************/

/**
 * @brief Create a new notifier after calling this function.
 *
 * @param dev[in] device descriptor.
 * @param phnotifier[out] a pointer to pointer for retrieving the new created notifier handle.
 * @param  flags[in] reserved for further use, pass 0 as well.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluCreateNotifier(MLUdev dev, MLUnotifier *phnotifier, unsigned int flags);

/**
 * @brief Destroy a notifier after calling this function.
 *
 * @param dev[in] device descriptor.
 * @param hnotifier[in] notifier handle created by calling mluCreateNotifier.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluDestroyNotifier(MLUdev dev, MLUnotifier hnotifier);

/**
 * @brief Wait until the specified notifier object is in the signaled state or exceeds the time-out interval.
 *
 * @param dev[in] device descriptor.
 * @param hnotifier[in] notifier handle created by calling mluCreateNotifier.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluWaitNotifier(MLUdev dev, MLUnotifier hnotifier);

/**
 * @brief Query the status of the notifier.
 *
 * @param dev[in] device descriptor.
 * @param hnotifier[in] notifier handle created by calling mluCreateNotifier.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 * Note : function returns 1 means that the notifier isn't executed.
 */
__CAMB_EXPORT extern MLUresult mluQueryNotifier(MLUdev dev, MLUnotifier hnotifier);

/**
 * @brief Place an notifier in specified queue.
 *
 * @param dev[in] device descriptor.
 * @param hnotifier[in] notifier handle created by calling mluCreateNotifier.
 * @param hqueue[in] queue handle created by calling mluCreateQueue or mluCreateQueueWithPriority.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluPlaceNotifyNotifier(MLUdev dev, MLUnotifier hnotifier, MLUqueue hqueue);

/**
 * @brief Computed the elapsed time between two notifier.
 *
 * @param dev[in] device descriptor.
 * @param ptv[out] time between hstart and hend.
 * @param hnotifier[in] start notifier handle created by calling mluCreateNotifier.
 * @param hnotifier[in] end notifier handle created by calling mluCreateNotifier.
 * @return MLU_SUCCESS if success,
 *         otherwise the error code is returned.
 */
__CAMB_EXPORT extern MLUresult mluNotifierElapsedTime(MLUdev dev, struct timeval *ptv, MLUnotifier hstart, MLUnotifier hend);

/**
 * @brief Get an ipc shared memory handle according to an allocated address.

   @param pdesc is a pointer to pointer for retrieving device descriptor.
   @param addr requires a MLU address which is already allocated.
   @param *pHandle returns a pointer to a ipc memory handle after calling.

   @Returns MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
 */

__CAMB_EXPORT extern MLUresult mluIpcGetMemHandle(MLUdev dev,MLUIpcMemHandle *pHandle,MLUaddr addr);

/**
 * @brief Open an ipc shared memory address from a ipc handle.

   @param pdesc is a pointer to pointer for retrieving device descriptor.
   @param handle requires an IPC handle which is already allocated.
   @param pdptr returns a pointer to pointer to a ipc shared memory.
   @param flag is reserved.

   @Returns MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/

__CAMB_EXPORT extern MLUresult mluIpcOpenMemHandle(MLUdev dev,MLUaddr *pdptr,MLUIpcMemHandle handle,unsigned int flag);

/**
 * @brief Close an ipc shared memory handle after using.
   @param pdesc is a pointer to pointer for retrieving device descriptor.
   @param dptr requires a pointer to ipc shared memory
    which needs to be closed and freed.

   @Returns MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
 */

__CAMB_EXPORT extern MLUresult mluIpcCloseMemHandle(MLUdev dev,MLUaddr dptr);
/**
 * @brief Get an ipc shared notifier handle after using.
   @param pdesc is a pointer to pointer for retrieving device descriptor.
   @param notifier requires a MLU notifier which is already allocated.
   @param *pHandle returns a pointer to an ipc notifier handle after calling.

   @Returns MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
 */

__CAMB_EXPORT extern MLUresult mluIpcGetNotifierHandle(MLUdev dev, MLUIpcNotifierHandle *pHandle,MLUnotifier notifier,unsigned int flag);

/**
 * @brief Open an ipc shared notifier address from a ipc handle.

   @param pdesc is a pointer to pointer for retrieving device descriptor.
   @param handle requires an IPC notifier handle which is already allocated.
   @param pnotifier returns a pointer to pointer to a ipc shared evevt.

   @Returns MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/

__CAMB_EXPORT extern MLUresult mluIpcOpenNotifierHandle(MLUdev dev, MLUIpcNotifierHandle Handle,MLUnotifier *pnotifier,unsigned int flag);

/**
 * @brief malloc a segment of host memory
   @param addr returns a pointer to a host addr which is allocated.
   @param size is the bytes count to be allocated.

   @Returns MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
 */
__CAMB_EXPORT extern MLUresult mluMallocHost(HOSTaddr *addr, uint64_t size);
/**
 * @brief free a segment of host memory
   @param addr requires a pointer to a host addr which is allocated.

   @Returns MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
 */
__CAMB_EXPORT extern MLUresult mluFreeHost(HOSTaddr addr);

/**
   @brief Retrieves the memory information of the current device.
   @param dev is a pointer to pointer for retrieving device descriptor.
   @param pdata is the memory info.

   @Returns  MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/
__CAMB_EXPORT extern MLUresult mluMemGetInfo(MLUdev dev, MLUMemoryInfo_t* pdata);

/**
   @brief Retrieves the power information of the current device.
   @param dev is a pointer to pointer for retrieving device descriptor.
   @param pdata is the power info.

   @Returns  MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/
__CAMB_EXPORT extern MLUresult mluPowerGetInfo(MLUdev dev, MLUPowerInfo_t* pdata);

/**
   @brief Retrieves the health state information of the current device.
   @param dev is a pointer to pointer for retrieving device descriptor.
   @param pdata is the health state info.

   @Returns  MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/
__CAMB_EXPORT extern MLUresult mluHealthStateGetInfo(MLUdev dev, MLUHealthStateInfo_t* pdata);


/**
   @brief Retrieves the ecc information of the current device.
   @param dev is a pointer to pointer for retrieving device descriptor.
   @param pdata is the ecc info.

   @Returns  MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/
__CAMB_EXPORT extern MLUresult mluECCGetInfo(MLUdev dev, MLUECCInfo_t* pdata);

/**
   @brief Retrieves the IPU util information of the current device.
   @param dev is a pointer to pointer for retrieving device descriptor.
   @param pdata is the IPU util info.

   @Returns  MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/
__CAMB_EXPORT extern MLUresult mluIPUUtilGetInfo(MLUdev dev, MLUIPUUtilInfo_t* pdata);

/**
   @brief Retrieves the freq information of the current device.
   @param dev is a pointer to pointer for retrieving device descriptor.
   @param pdata is the freq info.

   @Returns  MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/
__CAMB_EXPORT extern MLUresult mluFreqGetInfo(MLUdev dev, MLUFreqInfo_t* pdata);

/**
   @brief Retrieves the card information of the current device.
   @param dev is a pointer to pointer for retrieving device descriptor.
   @param pdata is the card info.

   @Returns  MLU_SUCCESS if succeeded,otherwise the errorcode is returned.
*/
__CAMB_EXPORT extern MLUresult mluCardGetInfo(MLUdev dev, MLUCardInfo_t* pdata);

/*
 * @brief Function to invoke atomic operation to hqueue.
 * @param dev[in] device descriptor.
 * @param hqueue[in] queue handle created by calling c20CreateQueue or c20CreateQueueWithPriority.
 * @param op_ptr[in] operation user address ptr.
 * @param op_data1[in] operation data.
 * @param op_data2[in] reserved.
 * @param type[in] operation type.
 * @param flag[in] operation flag 'enum MLUAtomicCompFlag' or 'enum MLUAtomicReqFlag'.

 * @Returns MLU_SUCCESS if succeeded, otherwise the errorcode is returned.
 */
__CAMB_EXPORT extern MLUresult mluQueueAtomicOperation(MLUdev dev, MLUqueue hqueue,
		uint64_t op_ptr, uint64_t op_data1,
		uint64_t op_data2, MLUAtomicOpType_t type, uint64_t flag);

/**
 * @brief Function to do atomic operation.
 * @param dev[in] device descriptor.
 * @param op_ptr[in] operation user address ptr.
 * @param op_data1[in] operation data.
 * @param op_data2[in] reserved.
 * @param type[in] operation type.
 * @param flag[in] operation flag 'enum MLUAtomicCompFlag' or 'enum MLUAtomicReqFlag'.

 * @Returns MLU_SUCCESS if succeeded, otherwise the errorcode is returned.
 *
 * Note : this function will not block.
 */
__CAMB_EXPORT extern MLUresult mluAtomicOperation(MLUdev dev,
		uint64_t op_ptr, uint64_t op_data1,
		uint64_t op_data2, MLUAtomicOpType_t type, uint64_t flag);

/**
 * @brief Read input host addr value.
 * @param dev[in] device descriptor.
 * @param op_ptr[in] operation user address ptr.

 * @Returns MLU_SUCCESS if succeeded, otherwise the errorcode is returned.
 *
 * Note : this function will not block.
 */
__CAMB_EXPORT extern MLUresult mluAtomicReadOps(MLUdev dev, uint64_t op_ptr, uint64_t *value);

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */
#endif  // CNDRV_API_H_
