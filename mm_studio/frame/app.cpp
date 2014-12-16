#include "frame.h"

class App: public wxApp
{
public:
    virtual bool OnInit();
};

DECLARE_APP (App)

IMPLEMENT_APP (App)

bool App::OnInit()
{
    // make sure to call this first
    wxInitAllImageHandlers();
    Frame* frame = new Frame (wxT ("Map Maching"));
    frame->Show (true);

    return true;
}
