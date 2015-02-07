#include "mm_node.h"
#include <node_buffer.h>

v8::Persistent<v8::Function> MMWrapper::constructor;

MMWrapper::MMWrapper() {
  solver_ = new MMSolver;
}

MMWrapper::~MMWrapper() {
  delete solver_;
}

void MMWrapper::Init(v8::Handle<v8::Object> exports) {
  NanScope();
  
  // function template
  v8::Local<v8::FunctionTemplate> tpl = NanNew<v8::FunctionTemplate>(New);
  tpl->SetClassName(NanNew("MMSolver"));

  // prototype template
  tpl->InstanceTemplate()->SetInternalFieldCount(2);

  // Prototype
  NODE_SET_PROTOTYPE_METHOD(tpl, "prepare", prepare);
  NODE_SET_PROTOTYPE_METHOD(tpl, "match", match);

  NanAssignPersistent(constructor, tpl->GetFunction());
  
  // export class
  exports->Set(NanNew("MMSolver"), tpl->GetFunction());
}

NAN_METHOD(MMWrapper::New) {
  NanScope();
  
  if (args.IsConstructCall()) {
    MMWrapper* obj = new MMWrapper();
    obj->Wrap(args.This());

    NanReturnValue(args.This());
  } else {
    const int argc = 0;
    v8::Local<v8::Value> argv[argc] = {};
    v8::Local<v8::Function> cons = NanNew<v8::Function>(constructor);

    NanReturnValue(cons->NewInstance(argc, argv));
  }
}

NAN_METHOD(MMWrapper::prepare) {
  NanScope();

  std::string node = *(v8::String::Utf8Value(args[0]->ToString()));
  std::string edge = *(v8::String::Utf8Value(args[1]->ToString()));
  std::string history = *(v8::String::Utf8Value(args[2]->ToString()));
  std::string out_dir = *(v8::String::Utf8Value(args[3]->ToString()));
  
  double xmin = args[4]->NumberValue();
  double ymin = args[5]->NumberValue();
  double xmax = args[6]->NumberValue();
  double ymax = args[7]->NumberValue();

    
  MMWrapper* obj = ObjectWrap::Unwrap<MMWrapper>(args.This());
  bool result = obj->solver_->Prepare(node, edge, history, out_dir, xmin, ymin, xmax, ymax);

  NanReturnValue(NanNew<v8::Boolean>(result));
}

NAN_METHOD(MMWrapper::match) {
  NanScope();

  std::string preprocess = *(v8::String::Utf8Value(args[0]->ToString()));
  std::string in = *(v8::String::Utf8Value(args[1]->ToString()));
  std::string out = *(v8::String::Utf8Value(args[2]->ToString()));
  std::string ground_truth = *(v8::String::Utf8Value(args[3]->ToString()));
    
  MMWrapper* obj = ObjectWrap::Unwrap<MMWrapper>(args.This());
  
  bool result = obj->solver_->Match(preprocess, in, out, ground_truth);

  NanReturnValue(NanNew<v8::Boolean>(result));
}
