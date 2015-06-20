#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class SentidosApp : public App {
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
	void update() override;
	void draw() override;
};

void SentidosApp::setup()
{
}

void SentidosApp::mouseDown( MouseEvent event )
{
}

void SentidosApp::update()
{
}

void SentidosApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) ); 
}

CINDER_APP( SentidosApp, RendererGl )
