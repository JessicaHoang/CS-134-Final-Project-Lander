//                                             (c) Kevin M. Smith  - 2018
#pragma once

#include "ofMain.h"
#include  "ofxAssimpModelLoader.h"
#include  "ofxGui.h"
#include "ParticleSystem.h"
#include "ParticleEmitter.h"
#include "box.h"
#include "ray.h"
#include "Octree.h"


class ofApp : public ofBaseApp {

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	void drawAxis(ofVec3f);
	void initLightingAndMaterials();
	void savePicture();
	void toggleWireframeMode();
	void togglePointsDisplay();
	void checkCollision(); 
	void checkFrontCollision(); 
	void checkBackCollision(); 
	void checkLeftCollision(); 
	void checkRightCollision();
	void playEngineSound(); 
	bool bSoundPlaying; 
	ofSoundPlayer engineSound;

	void loadVbo();
	// textures
	ofTexture  particleTex;
	// shaders
	ofVbo vbo;
	ofShader shader;

	ofColor particleColor;

	void toggleSelectTerrain();
	void toggleDrawBoundingBox();  
	void setCameraTarget();
	bool  doPointSelection();
	void drawBox(const Box &box);
	void drawMovingBox(const Box &box, const ofVec3f &offset); 
	Box meshBounds(const ofMesh &);
	void subDivideBox8(const Box &b, vector<Box> & boxList);

	bool mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point);

	Box boundingBox;
	Box landerBox;         
	ofxAssimpModelLoader  rover, terrain;

	bool bPointSelected;
	bool bRoverLoaded;
	bool bTerrainSelected;
	bool bCollision; 
	bool bSideCollision;  
	bool bDrawBounding;

	ofVec3f selectedPoint;
	ofVec3f pointOfIntersection;

	ofVec3f pointOfContact;

	ofVec3f frontPointOfContact;
	ofVec3f backPointOfContact;
	ofVec3f rightPointOfContact;
	ofVec3f leftPointOfContact;
	Octree  octree;
	TreeNode selectedNode;



	const float selectionRange = 4.0;

	ofEasyCam cam;
	ofxAssimpModelLoader lander, landerTextured; 
	ofLight keyLight;
	ofImage backgroundImage;
	ofCamera *theCam = NULL;
	ofCamera trackingCam; 
	ofCamera frontCam;
	ofCamera downCam;

	bool bAltKeyDown;
	bool bCtrlKeyDown;
	bool bWireframe;
	bool bDisplayPoints;

	bool bBackgroundLoaded = false;
	bool bLanderLoaded = false;

	ParticleEmitter emitter;
	ParticleEmitter engineEmitter;
	ParticleEmitter engineEmitter2;
	TurbulenceForce* turbForce;
	GravityForce* gravityForce;
	ThrustForce* thrustForceLunar;

	TurbulenceForce* turbForce2;
	GravityForce* gravityForce2;
	ThrustForce* thrustForceLunar2;

	ImpulseForce* impulseForce;

	ofxPanel gui;
	ofxFloatSlider gravity;
	ofxFloatSlider restitution;


};