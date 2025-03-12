#define LOGSIZE (6 * 5)

class DataLog {
  private:
//    float log[LOGSIZE] = {0};
    float *log; 
    int logsize;
    int index = 0;
    int current = -1;
    bool first_cycle = true;
  public:
    DataLog(int numData = LOGSIZE);
    ~DataLog();
    float average();
    float latest();
    int add(float v);
};

