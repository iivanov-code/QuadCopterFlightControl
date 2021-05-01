// RunningAverage class
// based on RunningAverage library for Arduino
// source:  http://playground.arduino.cc/Main/RunningAverage

#ifndef MOVING_AVERAGE
#define MOVING_AVERAGE

template <typename T> class MovingAverage
{
  public:
    MovingAverage(int count = 3);
    ~MovingAverage();
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

template class MovingAverage<int>;

#endif
