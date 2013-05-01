#include "quick-sort.h"
#include <iostream>
#include "ns3/log.h"
#include "ns3/object.h"


namespace ns3
{

NS_OBJECT_ENSURE_REGISTERED (QuickSort);


TypeId QuickSort::GetTypeId(void)
{
  static TypeId tid = TypeId ("ns3::QuickSort")
    .SetParent<Object> ()
    .AddConstructor<QuickSort> ()
    ;
  return tid;
}

QuickSort::QuickSort ()
{
  sortingArrayDouble = new std::vector<double> ();
}

  
QuickSort::QuickSort(std::vector<double> *sortArray)
{
  int startIndex = 0;
  int endIndex = 0;
                          
  endIndex = (int)(*sortArray).size() - 1;
  
  QuickSort(sortArray, startIndex, endIndex);
}
  

QuickSort::QuickSort(std::vector<uint32_t> *sortArray)
{
  int startIndex = 0;
  int endIndex = 0;
                          
  endIndex = (int)(*sortArray).size() - 1;
  
  QuickSort(sortArray, startIndex, endIndex);
}



QuickSort::QuickSort(std::vector<double> *sortArray, int startIndex, int endIndex)
{
  if(startIndex >= endIndex)
  {
    return;
  }
  
  // *initialize* the array
  sortingArrayDouble = sortArray;
  
  // For instance: 1..n
  //left = startIndex;
  //right = endIndex;
  
  if(startIndex < endIndex)
  {
    // get pivot
    int pivot = Partition(sortingArrayDouble, startIndex, endIndex);
    
    // sort left side
    QuickSort(sortingArrayDouble, startIndex, (pivot-1));
    
    // sort right side
    QuickSort(sortingArrayDouble, (pivot+1), endIndex);
  }
}

QuickSort::QuickSort(std::vector<uint32_t> *sortArray, int startIndex, int endIndex)
{
  if(startIndex >= endIndex)
  {
    return;
  }
  
  // *initialize* the array
  sortingArrayUint = sortArray;
  
  // For instance: 1..n
  //left = startIndex;
  //right = endIndex;
  
  if(startIndex < endIndex)
  {
    // get pivot
    int pivot = Partition(sortingArrayUint, startIndex, endIndex);
    
    // sort left side
    QuickSort(sortingArrayUint, startIndex, (pivot-1));
    
    // sort right side
    QuickSort(sortingArrayUint, (pivot+1), endIndex);
  }
}


int QuickSort::Partition(std::vector<double> *sortArray, int startIndex, int endIndex)
{       
  // initially this start - 1 when startIndex is 0.
  int l = startIndex - 1;
  int k = endIndex;

  for(int i = startIndex; i <= k - 1; i++)
  {
    // Go until you find a value smaller than the last value.
    if((*sortArray)[i] <= (*sortArray)[k])
    {
      // increment l
      l++;
    
      // swap i and j
      // NOTE: this is supposed to swap j with itself the first time.
      Swap(l, i);       
    }
  }   
 
  // when loop is finished, swap
  Swap(l + 1, k);
  
  return l + 1;
}

int QuickSort::Partition(std::vector<uint32_t> *sortArray, int startIndex, int endIndex)
{       
  // initially this start - 1 when startIndex is 0.
  int l = startIndex - 1;
  int k = endIndex;

  for(int i = startIndex; i <= k - 1; i++)
  {
    // Go until you find a value smaller than the last value.
    if((*sortArray)[i] <= (*sortArray)[k])
    {
      // increment l
      l++;
    
      // swap i and j
      // NOTE: this is supposed to swap j with itself the first time.
      SwapUint(l, i);       
    }
  }   
 
  // when loop is finished, swap
  SwapUint(l + 1, k);
  
  return l + 1;
}


void QuickSort::Swap(int l, int k)
{
  // create temp variable
  double tmp;
  
  // store first element in temp
  tmp =(*sortingArrayDouble)[l];
  
  // swap second element to first element
  (*sortingArrayDouble)[l] = (*sortingArrayDouble)[k];
  
  // put temp variable in second element
  (*sortingArrayDouble)[k] = tmp;
}


void QuickSort::SwapUint(int l, int k)
{
  // create temp variable
  uint32_t tmp;
  
  // store first element in temp
  tmp =(*sortingArrayUint)[l];
  
  // swap second element to first element
  (*sortingArrayUint)[l] = (*sortingArrayUint)[k];
  
  // put temp variable in second element
  (*sortingArrayUint)[k] = tmp;
}

}
