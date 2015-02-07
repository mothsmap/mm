#include <node.h>
#include "mm_node.h"

void initAll(v8::Handle<v8::Object> exports) {
	MMWrapper::Init(exports);
}

NODE_MODULE(MMSolver, initAll)

