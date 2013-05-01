#ifndef _QUICKSORT_H
#define _QUICKSORT_H
#include "ns3/object.h"
#include <vector>
namespace ns3{
/*
 *     QuickSort Algorithm using Templates
 *      
 *     Algorithm was adapted from:
 *     Introduction to Algorithms
 *     By Thomas H. Cormen, Charles E. Leiserson, Ronald L. Rivest, Clifford Stein
 *     Contributor Thomas H. Cormen
 *     Edition: 2, illustrated
 *     Published by MIT Press, 2001
 *     ISBN 0262032937, 9780262032933
 *     1180 pages
 *     Accessed March 2009
 *     via http://books.google.com/books?id=NLngYyWFl_YC
 *     See pages 145-147
 */                                                                               

 //template <typename T>
 class QuickSort: public Object
 {
 public:
   static TypeId GetTypeId (void);
   std::vector<double> *sortingArrayDouble;
   std::vector<uint32_t> *sortingArrayUint;
   int left;
   int right;
   //Constructors
   QuickSort();


   QuickSort(std::vector<double> *sortArray);
   QuickSort(std::vector<uint32_t> *sortArray);


   QuickSort(std::vector<double> *sortArray, int startIndex, int endIndex);
   QuickSort(std::vector<uint32_t> *sortArray, int startIndex, int endIndex);
   
  /**********************************************************************
  * Partition and Swap Functions
  **********************************************************************/






   int Partition(std::vector<double> *sortArray, int startIndex, int endIndex);
   int Partition(std::vector<uint32_t> *sortArray, int startIndex, int endIndex);

   void Swap(int l, int k);
   void SwapUint(int l, int k);
  
 };
}
#endif
