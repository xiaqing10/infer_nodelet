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
#ifndef _CN_CODEC_DEFINE_H_
#define _CN_CODEC_DEFINE_H_

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#include <stdbool.h>
#include <stdint.h>

#if defined(WIN32) || defined(WINDOWS)
#ifdef USE_CNCODEC_DLL
#ifdef CNCODEC_DLL_EXPORTS
#define CNCODEC_DLL_API __declspec(dllexport)
#else
#define CNCODEC_DLL_API __declspec(dllimport)
#endif /* CNCODEC_DLL_EXPORTS */
#else
#define CNCODEC_DLL_API
#endif /* USE_CNCODEC_DLL */
#else
#define CNCODEC_DLL_API
#endif /* WIN32 || WINDOWS */

#ifndef __CAMBRICON_CNCODEC_TYPES_H__
#define __CAMBRICON_CNCODEC_TYPES_H__
#if defined(WIN32) || defined(WINDOWS)
typedef unsigned __int64  u64_t;
typedef unsigned __int32  u32_t;
typedef unsigned __int16  u16_t;
typedef unsigned __int8   u8_t;
typedef __int64           i64_t;
typedef signed __int32    i32_t;
typedef signed __int16    i16_t;
typedef signed __int8     i8_t;
typedef unsigned __int32  bool_t;
#else
typedef uint64_t          u64_t;
typedef uint32_t          u32_t;
typedef uint16_t          u16_t;
typedef uint8_t           u8_t;
typedef int64_t           i64_t;
typedef int32_t           i32_t;
typedef int16_t           i16_t;
typedef int8_t            i8_t;
typedef uint32_t          bool_t;
#endif /*WIN32||WINDOWS*/

#endif /*__CAMBRICON_CNCODEC_TYPES_H__*/

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _CN_CODEC_DEFINE_H_ */