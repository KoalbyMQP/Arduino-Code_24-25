#ifndef ArduinoPoppy_h
#define ArduinoPoppy_h
#include <Herkulex.h>

// the #include statment and code go here...
class ArduinoPoppy {
  public:
    ArduinoPoppy();
    void Setup();
    int ReadCommand();
    void Initialize(int idArr[][4]);
    void GetPosition();
    void SetPosition(int idArr[][4]);
    void SetPositionT(int idArr[][4]);
    void ArmMirror(int idArr[][4], int mirrorArray[4][2], bool armMirrorModeOn, int lastMirror);
    void PuppetMaster();
  private:


};

#endif
