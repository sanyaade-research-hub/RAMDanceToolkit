#include "ramUtils.h"


#pragma mark - ramColor
const ofColor ramColor::RED_NORMAL		= ofColor::fromHex(0xff6666);
const ofColor ramColor::RED_DEEP		= ofColor::fromHex(0x993333);
const ofColor ramColor::RED_LIGHT	 	= ofColor::fromHex(0xff9898);

const ofColor ramColor::GREEN_NORMAL 	= ofColor::fromHex(0x66cc33);
const ofColor ramColor::GREEN_DEEP		= ofColor::fromHex(0x339900);
const ofColor ramColor::GREEN_LIGHT 	= ofColor::fromHex(0x99cc99);

const ofColor ramColor::BLUE_NORMAL 	= ofColor::fromHex(0x0099cc);
const ofColor ramColor::BLUE_DEEP   	= ofColor::fromHex(0x003366);
const ofColor ramColor::BLUE_LIGHT  	= ofColor::fromHex(0x99cccc);

const ofColor ramColor::YELLOW_NORMAL	= ofColor::fromHex(0xffcc00);
const ofColor ramColor::YELLOW_DEEP 	= ofColor::fromHex(0xcc9900);
const ofColor ramColor::YELLOW_LIGHT	= ofColor::fromHex(0xffff00);

const ofColor ramColor::BLACK			= ofColor::fromHex(0x000000);
const ofColor ramColor::GRAY			= ofColor::fromHex(0x666666);
const ofColor ramColor::WHITE			= ofColor::fromHex(0xffffff);

//--------------------------------------------------------------
void ramPushAll()
{
    ofPushView();
    
    glPushAttrib(GL_ALL_ATTRIB_BITS);
    
    GLint matrixMode;
    glGetIntegerv(GL_MATRIX_MODE, &matrixMode);
    
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glMatrixMode(GL_TEXTURE);
    glPushMatrix();
    glMatrixMode(GL_COLOR);
    glPushMatrix();
    
    glMatrixMode(matrixMode);
    
    ofPushStyle();
}

//--------------------------------------------------------------
void ramPopAll()
{
    ofPopStyle();
    
    GLint matrixMode;
    glGetIntegerv(GL_MATRIX_MODE, &matrixMode);
    
    glMatrixMode(GL_COLOR);
    glPopMatrix();
    glMatrixMode(GL_TEXTURE);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    
    glMatrixMode(matrixMode);
    
    glPopAttrib();
    
    ofPopView();
}