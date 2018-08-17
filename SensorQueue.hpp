#ifndef _SENSORQUEUE_H_
#define _SENSORQUEUE_H_

#include "mbed.h"
#include "mbed_events.h"
#include <list>
#include <cstdlib>
#include <cstring>

template <class T>
class SensorQueue {
  private:
    uint16_t blk_length;
    uint16_t max_pool_blks;
    uint16_t total_blks;
    uint16_t blk_offset; //The index within an individual block
    Thread eventThread;
    Semaphore *buffer_lock;
    EventQueue queue;


    void (*callback_func)(void);
    std::list<T*> block_list;
    T* newBlock(void);
    void append_helper(T elem);

  public:
    SensorQueue(uint32_t length, uint16_t blk_length, uint16_t max_pool_blks = 1);
    void append(T elem);  //called by interrupt
    void setCallBack(void (*_fun_ptr)(void));  //notify when a segment is full
    void copyTo(void *ptr, bool adv_frame = true);  //fill the ptr with the newest elements
    void printStates(void);
    ~SensorQueue();
};

template <class T>
void SensorQueue<T>::printStates(void) {
  printf("blen: %d, bmax: %d, bttl: %d,\r\n", blk_length, max_pool_blks, total_blks);
  printf("len: %d, nblocks: %d, boffet: %d\r\n", block_list.size() * blk_length, block_list.size(), blk_offset);
}

template <class T>
SensorQueue<T>::SensorQueue(uint32_t length, uint16_t _blk_length, uint16_t _max_pool_blks) {

  blk_length = _blk_length;
  max_pool_blks = _max_pool_blks;
  
  total_blks = length / blk_length;
  if(length > (total_blks * blk_length)) total_blks += 1;
  blk_offset = 0;

  eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
  buffer_lock = new Semaphore(1);
  //Init Block
  newBlock();

  callback_func = NULL;  
}

template <class T>
T* SensorQueue<T>::newBlock(void) {
  blk_offset = 0;
  T* new_blk = (T*) malloc(sizeof(T) * blk_length);
  block_list.push_back(new_blk);

  return new_blk;
}

template <class T>
void SensorQueue<T>::setCallBack(void (*_fun_ptr)(void)) {
  callback_func = _fun_ptr;
}

template <class T>
void SensorQueue<T>::append_helper(T elem) {
  buffer_lock->wait();
  if(blk_offset >= blk_length) {
    if(block_list.size() < max_pool_blks + total_blks) {
      //(*callback_func)();
      if(callback_func != NULL) {
        queue.call(*callback_func);  //NT: change this to mbed os default queue maybe?
        //queue.dispatch(0);
      }
    } else {
      printf("warning: max_pool_blks exceeded\r\n");
      block_list.pop_front();
    }

    newBlock();  //new block and resets blk_offset
  }

  T* current_blk = block_list.back();
  current_blk[blk_offset] = elem;
  blk_offset += 1;
  buffer_lock->release();
}

template <class T>
void SensorQueue<T>::append(T elem) {
  queue.call(this, &SensorQueue<T>::append_helper, elem);
  //queue.dispatch(0); //returns immediately
}

template <class T>
void SensorQueue<T>::copyTo(void *ptr, bool adv_frame) {
  buffer_lock->wait();
  auto it = block_list.begin();
  for(int i = 0; i < total_blks; i++) {
    //printf("offset is: %d\r\n", i * blk_length);
    memcpy((void*)(ptr + i * blk_length * sizeof(T)), (void*) *it, sizeof(T) * blk_length);
    it++;
  }

  if(adv_frame && block_list.size() > total_blks) {
    free(block_list.front());
    block_list.pop_front();
  }
  buffer_lock->release();
}

template <class T>
SensorQueue<T>::~SensorQueue() {
  delete buffer_lock;
}
template <class T>
bool compare_test(T* input, list<T> ref, int n) {
  int i = 0;
  bool result = true;
  auto it = ref.begin();
  for(int i = 0; i < n; i++) {
    if(*it != input[i]) {
      result = false;
      break;
    }
    it++;
  }

  if(!result) {
    printf("=============================\n");
    it = ref.begin();
    for(int i = 0; i < n; i++) {
      printf("reference: ");
      printf("%d ", *it);
      it++;
    }
    printf("\n");
    for(i = 0; i < n; i++) {
      printf("result   : ");
      printf("%d ", input[i]);
    }
    printf("\n");
  } else {
    printf("ok\r\n");
  }

  return result;
}

void SensorQueue_Test(int window_size, int incre_step, int n_max_pool) {
  SensorQueue<int> buff(window_size, incre_step, n_max_pool);
  list<int> ref;
  int result[window_size];

  printf("=============================\r\n");
  printf("Constructing initial test data\r\n");
  for(int i = 0; i < 12; i++) {
    ref.push_back(i);
    buff.append(i);
  }
  buff.printStates();
  //l:12 b:3

  printf("=============================\r\n");
  printf("test 1: ");
  buff.copyTo(result);
  if(!compare_test(result, ref, window_size)) { printf("test failed"); exit(-1); }
  buff.printStates();
  //l:12 b:3

  printf("=============================\r\n");
  printf("Constructing 2nd test data\r\n");
  ref.clear();
  int result2[window_size];
//8 2 2
  for(int i = 0; i < 17; i++) {
    ref.push_back(i * -1);
    buff.append(i * -1);
  }
  int n_pop = 17 - (window_size + n_max_pool * incre_step);
  if(n_pop > 0 && n_pop % incre_step != 0) n_pop = n_pop - (n_pop % incre_step) + incre_step; //round up
  for(int i = 0; i < n_pop; i++) {
    ref.pop_front();
  }
  buff.printStates();
  //l:16 b:4

  printf("=============================\r\n");
  printf("test 2: ");
  buff.copyTo(result2);
  if(!compare_test(result2, ref, window_size)) { printf("test failed"); exit(-1); }
  for(int i = 0; i < incre_step; i++) { ref.pop_front(); }
  buff.printStates();

  printf("=============================\r\n");
  //l:12 b:3
  printf("test 3: ");
  buff.copyTo(result);
  if(!compare_test(result, ref, window_size)) { printf("test failed"); exit(-1); }
  buff.printStates();
  printf("=============================\r\n");
  //l:12 b:3

  // printf("=============================\r\n");
  // ref.clear();
  // printf("Constructing overflow test data\r\n");
  // //overflows the buffer
  // for(int i = 0; i < 35; i++) {
  //   buff.append(-1024);
  // }

  // for(int i = 0; i < window_size; i++) {
  //   ref.push_back(i * -1);
  //   buff.append(i * -1);
  // }

  // for(int i = 0; i < incre_step * n_max_pool; i++) {
  //   buff.append(0);
  // } 

  // buff.copyTo(result);
  // buff.printStates();
  // if(!compare_test(result, ref, window_size)) { printf("test failed"); exit(-1); }

}
#endif