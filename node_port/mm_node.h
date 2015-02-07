#ifndef __mm__node__
#define __mm__node__

#include "mm.h"
#include <nan.h>

class MMWrapper: public node::ObjectWrap {
private:
    MMSolver* solver_;
    
    explicit MMWrapper();
    ~MMWrapper();
    
    static v8::Persistent<v8::Function> constructor;
    
    static NAN_METHOD(New);
    static NAN_METHOD(prepare);
    static NAN_METHOD(match);
    
public:
    static void Init(v8::Handle<v8::Object> exports); 
};

#endif // __mm__node__
