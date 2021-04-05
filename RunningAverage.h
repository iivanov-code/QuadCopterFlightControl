// RunningAverage class
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage

#ifndef RUNNING_AVERAGE
#define RUNNING_AVERAGE

template <typename T> class RunningAverage
{
  public:
    RunningAverage(int count = 3);
    ~RunningAverage();
    void Clear();
    void Add(T);
    T GetAverage() const;
    void FillValue(T, int);
    int getSize();
    T PrevAverageValue = Zero;
  protected:
    int Size;
    int Count;
    int _idx;
    T SUM;
    T* dataArray;
    static T Zero;
};

template class RunningAverage<int>;

#endif
