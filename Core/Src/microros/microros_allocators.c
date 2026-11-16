/*
 * ===== 中文說明 =====
 * micro-ROS 的記憶體配置函式。microros_app.c 會把它們設成 rcutils 的預設 allocator，
 * 所以 micro-ROS 內部所有 malloc / free 都會走到這裡，
 * 再轉給 custom_memory_manager.c 的專用 heap。
 * 另外記錄兩個除錯用的數字，可以在 debugger 的 Live Expressions 觀察：
 *   absoluteUsedMemory  累計申請過的總量（只增不減）。
 *   usedMemory          目前還在使用的量。
 *
 * 函式：
 *   microros_allocate()       相當於 malloc()。
 *   microros_deallocate()     相當於 free()，傳 NULL 時什麼都不做。
 *   microros_reallocate()     相當於 realloc()，傳 NULL 時等同 malloc()。
 *   microros_zero_allocate()  相當於 calloc()，配置後清成 0。
 */

#include <unistd.h>
#include "cmsis_os.h"

int absoluteUsedMemory = 0;
int usedMemory = 0;

void *pvPortMallocMicroROS( size_t xWantedSize );
void vPortFreeMicroROS( void *pv );
void *pvPortReallocMicroROS( void *pv, size_t xWantedSize );
size_t getBlockSize( void *pv );
void *pvPortCallocMicroROS( size_t num, size_t xWantedSize );

void * microros_allocate(size_t size, void * state){
  (void) state;
  // printf("-- Alloc %d (prev: %d B)\n",size, xPortGetFreeHeapSize());
  absoluteUsedMemory += size;
  usedMemory += size;
  return pvPortMallocMicroROS(size);
}

void microros_deallocate(void * pointer, void * state){
  (void) state;
  // printf("-- Free %d (prev: %d B)\n",getBlockSize(pointer), xPortGetFreeHeapSize());
  if (NULL != pointer){
    usedMemory -= getBlockSize(pointer);
    vPortFreeMicroROS(pointer);
  }
}

void * microros_reallocate(void * pointer, size_t size, void * state){
  (void) state;
  // printf("-- Realloc %d -> %d (prev: %d B)\n",getBlockSize(pointer),size, xPortGetFreeHeapSize());
  absoluteUsedMemory += size;
  usedMemory += size;
  if (NULL == pointer){
    return pvPortMallocMicroROS(size);
  } else {
    usedMemory -= getBlockSize(pointer);
    return pvPortReallocMicroROS(pointer,size);
  }
}

void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state){
  (void) state;
  // printf("-- Calloc %d x %d = %d -> (prev: %d B)\n",number_of_elements,size_of_element, number_of_elements*size_of_element, xPortGetFreeHeapSize());
  absoluteUsedMemory += number_of_elements*size_of_element;
  usedMemory += number_of_elements*size_of_element;
  return pvPortCallocMicroROS(number_of_elements,size_of_element);
}