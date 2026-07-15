/*************************************************************************
 * Copyright (C) [2019] by Cambricon, Inc.
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *************************************************************************/
#ifndef _CN_CODEC_MEMORY_H_
#define _CN_CODEC_MEMORY_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include "cn_codec_define.h"

/**
 * @brief Map device addr returned by a previous call to cnrtMallocFrameBuffer
 *        into host addr in userspace.
 *
 * @param host_ptr[out] maped address of host.
 * @param dev_ptr[in]  address of device.
 * @param size[in]  memory size
 * @return CNRT_RET_SUCCESS if success,
 *         otherwise the error code(refer to cnrtRet_t) is returned.
 */
extern CNCODEC_DLL_API i32_t cncodecMap(void **hostPtr, u64_t devPtr, u32_t size);

/**
 * @brief Unmap the memory space pointed by host_ptr, which must
 *        be returned by a previous call to cncodecMap.
 *

 * @param host_ptr[in] point to the memory to be free
 * @param size[in]  memory size
 * @return CNRT_RET_SUCCESS if success,
 *         otherwise the error code(refer to cnrtRet_t) is returned.
 */
extern CNCODEC_DLL_API i32_t cncodecUnmap(void *hostPtr, u32_t size);

/**
 * @brief Flush Cache, sync data from host_ptr to dev_ptr

 *
 * @param host_ptr[in] maped address of host.
 * @param size[in]  memory size
 * @return CNRT_RET_SUCCESS if success,
 *         otherwise the error code(refer to cnrtRet_t) is returned.
 */
extern CNCODEC_DLL_API i32_t cncodecFlushCache(void *hostPtr, u32_t size);

/**
 * @brief Invaid Cache, sync data from dev_ptr to host_ptr

 *
 * @param host_ptr[in] maped address of host.
 * @param size[in]  memory size
 * @return CNRT_RET_SUCCESS if success,
 *         otherwise the error code(refer to cnrtRet_t) is returned.
 */
extern CNCODEC_DLL_API i32_t cncodecInvalidateCache(void *hostPtr, u32_t size);


#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _CN_CODEC_MEMORY_H_ */
