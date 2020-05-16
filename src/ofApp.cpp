#include <glm/gtx/intersect.hpp>

//--------------------------------------------------------------
//
//  CS134 - Game Development
//
//  3D Model Drag and Drop and Ray Tracing Selection - startup scene
// 
//  This is an openFrameworks 3D scene that includes an EasyCam
//  and example 3D geometry which I have modelled in Houdini that
//  represents lunar terrain.
//
//  You will use this source file (and include file) as a starting point
// 
//  Please do not modify any of the keymappings.  I would like 
//  the input interface to be the same for each student's 
//  work.  Please also add your name/date below.
//
//  Please document/comment all of your work !
//  Have Fun !!
//
//  Kevin Smith   10-20-19
//
//  Student Name:   < Your Name goes Here >
//  Date: <date of last version>


#include "ofApp.h"
#include "Util.h"
#include "Octree.h"
#include "box.h"



//--------------------------------------------------------------
// setup scene, lighting, state and load geometry
//
void ofApp::setup() {

	//texture loading
	ofDisableArbTex();

	if (!ofLoadImage(particleTex, "images/dot.png"))
	{
		cout << "Particle Texture file cannot be found images/dot.png" << endl;
		ofExit();
	}

//load shader
#ifdef TARGET_OPENGLES
	shader.load("shades_gles/shader");
#else
	shader.load("shaders/shader");
#endif

	particleColor = ofColor(200, 100, 50);

	gui.setup();
	gui.add(gravity.setup("Gravity", 0, 0, 2));
	gui.add(restitution.setup("Restitution", 0, 0, 1));
	bWireframe = false;
	bDisplayPoints = false;
	bAltKeyDown = false;
	bCtrlKeyDown = false;
	bLanderLoaded = false;

	//load booleans
	bRoverLoaded = false;
	bTerrainSelected = true;
	bCollision = false;
	bSideCollision = false;
	bSoundPlaying = false;

	
	engineSound.load("sounds/engine.mp3");
	engineSound.setLoop(true);

	bgSound.load("sounds/comet.mp3");
	bgSound.setLoop(true);
	bgSound.play();
	cam.setDistance(15);
	cam.setNearClip(.2);
	cam.setFov(66);

	ofSetVerticalSync(true);
	ofEnableSmoothing();
	ofEnableDepthTest();

	cam.disableMouseInput();

	//current camera
	theCam = &cam;

	ofSetVerticalSync(true);
	ofEnableSmoothing();
	ofEnableDepthTest();

	//background image
	bBackgroundLoaded = backgroundImage.load("images/space.png");

	//set lighting
	initLightingAndMaterials();

	//set terrain and bounding box
	terrain.loadModel("geo/mars-5k.obj");
	terrain.setScaleNormalization(false);

	

	boundingBox = meshBounds(terrain.getMesh(0));
	cout << "number of meshes land: " << terrain.getMeshCount() << endl;



	

	// create Octree for terrain
	//
	//Build octree with 9 levels
	octree.create(terrain.getMesh(0), 7);
	terrain.setRotation(0, 180, 0, 0, 1);
	if (lander.loadModel("geo/Rocket.obj"))
	{
		lander.setScaleNormalization(false);
		lander.setScale(1, 1, 1);
		lander.setPosition(0, 150, 0);
		lander.setRotation(0, 90, 0, 0, 1);
		landerBox = meshBounds(lander.getMesh(0));
		bLanderLoaded = true;
		
	}
	else
	{
		cout << "Error: cannot load model geo/lander.obj" << endl;
		ofExit(0);
	}

	//Camera setup - Jessica Hoang

	trackingCam.setPosition(ofVec3f(0, 10, 30));
	trackingCam.lookAt(lander.getPosition());

	
	downCam.setPosition(lander.getPosition());
	downCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));


	frontCam.setPosition(lander.getPosition());
	float forward = lander.getPosition().x == 0 ? 1 : lander.getPosition().x;
	frontCam.lookAt(ofVec3f(abs(forward), lander.getPosition().y, lander.getPosition().z) * ofVec3f(-2, 1, 1));

	///////////////////Set up emitters
	
	//Create forces Justin
	turbForce = new TurbulenceForce(ofVec3f(0, 0, 0), ofVec3f(0, 0, 0));
	gravityForce = new GravityForce(ofVec3f(0, -gravity, 0));
	thrustForceLunar = new ThrustForce(ofVec3f(0, 0, 0));
	impulseForce = new ImpulseForce();

	turbForce2 = new TurbulenceForce(ofVec3f(-3, -3, -3), ofVec3f(3, 3, 3));
	gravityForce2 = new GravityForce(ofVec3f(0, 0, 0));
	thrustForceLunar2 = new ThrustForce(ofVec3f(10, 0, 0));

	//Emitter settings - Jessica Hoang
	emitter.setVelocity(ofVec3f(0, 0, 0));
	emitter.setOneShot(true);
	emitter.setEmitterType(RadialEmitter);
	emitter.setGroupSize(1);
	emitter.setLifespan(100000);

	engineEmitter.setVelocity(ofVec3f(0, 0, 0));
	engineEmitter.setOneShot(true);
	engineEmitter.setGroupSize(30);
	engineEmitter.setRandomLife(true);
	engineEmitter.setLifespanRange(ofVec2f(1, 1));
	engineEmitter.setPosition(lander.getPosition() + ofVec3f(4, -3, 2));
	engineEmitter.setEmitterType(DiscEmitter);

	engineEmitter2.setVelocity(ofVec3f(0, 0, 0));
	engineEmitter2.setOneShot(true);
	engineEmitter2.setGroupSize(30);
	engineEmitter2.setRandomLife(true);
	engineEmitter2.setLifespanRange(ofVec2f(1, 1));
	engineEmitter2.setPosition(lander.getPosition() + ofVec3f(4, -3, -2));
	engineEmitter2.setEmitterType(DiscEmitter);

	//Adding forces - Jessica Hoang
	emitter.sys->addForce(turbForce);
	emitter.sys->addForce(gravityForce);
	emitter.sys->addForce(thrustForceLunar);
	emitter.sys->addForce(impulseForce);
	emitter.setPosition(lander.getPosition());

	engineEmitter.sys->addForce(turbForce2);
	engineEmitter.sys->addForce(gravityForce2);
	engineEmitter.sys->addForce(thrustForceLunar2);

	engineEmitter2.sys->addForce(turbForce2);
	engineEmitter2.sys->addForce(gravityForce2);
	engineEmitter2.sys->addForce(thrustForceLunar2);

	emitter.spawn(ofGetElapsedTimef());
	
	//Setup game elements - Jessica Hoang
	gasoline = 1000;
	score = 0;
	bGameActive = false;
	showLostMessage = false;
	showWinMessage = false;
	win = false;
	
}

//--------------------------------------------------------------
// incrementally update scene (animation)
//
void ofApp::update() {

	// Jessica Hoang
	if (bGameActive && win)
	{
		bGameActive = false;
		showWinMessage = true;
	}
	if (bGameActive && gasoline <= 0)
	{
		bGameActive = false;
		showLostMessage = true;
	}
	if (bGameActive)
	{
		//update gravity - Jessica Hoang
		if (!bCollision)
		{
			gravityForce->set(ofVec3f(0, -gravity, 0));
		}

		//Update engine emitters
		emitter.sys->update();
		if (emitter.sys->particles.size() > 0)
		{
			lander.setPosition(emitter.sys->particles[0].position.x, emitter.sys->particles[0].position.y, emitter.sys->particles[0].position.z);
		}
		engineEmitter.setPosition(lander.getPosition() + ofVec3f(4, -3, 2));
		engineEmitter.update();

		engineEmitter2.setPosition(lander.getPosition() + ofVec3f(4, -3, 2));
		engineEmitter2.update();

		//check bottom collision if lander is moving downwards Justin
		if (emitter.sys->particles[0].velocity.y < .6)
		{
			checkCollision();
		}

		//Check front bumper Justin
		if (emitter.sys->particles[0].velocity.x < -0.6)
		{
			checkFrontCollision();
		}

		//Check back bumper Justin
		if (emitter.sys->particles[0].velocity.x > .6)
		{
			checkBackCollision();
		}

		//Check left bumper Justin
		if (emitter.sys->particles[0].velocity.z < -0.6)
		{
			checkLeftCollision();
		}

		//Check right bumper Justin
		if (emitter.sys->particles[0].velocity.z > 0.6)
		{
			checkRightCollision();
		}

		//Update camera position Justin
		trackingCam.lookAt(lander.getPosition());
		downCam.setPosition(lander.getPosition());
		downCam.lookAt(lander.getPosition() * ofVec3f(1, 0, 1));

		frontCam.setPosition(lander.getPosition());
		float forward = lander.getPosition().x == 0 ? 1 : lander.getPosition().x;
		frontCam.lookAt(ofVec3f(abs(forward), lander.getPosition().y, lander.getPosition().z) * ofVec3f(-2, 1, 1));
	}
	

}

//---------------------------------------------------------------------------------------------------------------------------

// load vertex buffer in preparation for rendering Justin
void ofApp::loadVbo() {
	if (engineEmitter.sys->particles.size() < 1) return;

	vector<ofVec3f> sizes;
	vector<ofVec3f> points;
	for (int i = 0; i < engineEmitter.sys->particles.size(); i++) {
		points.push_back(engineEmitter.sys->particles[i].position);
		sizes.push_back(ofVec3f(10));
	}
	for (int i = 0; i < engineEmitter2.sys->particles.size(); i++) {
		points.push_back(engineEmitter2.sys->particles[i].position);
		sizes.push_back(ofVec3f(10));
	}
	//upload the data to the vbo
	int total = (int)points.size();
	vbo.clear();
	vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
	vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
}
//--------------------------------------------------------------
void ofApp::draw() {

	if (bBackgroundLoaded)
	{
		ofPushMatrix();
		ofDisableDepthTest();
		ofSetColor(60, 60, 60);
		ofScale(2, 2);
		backgroundImage.draw(-100, -50);
		ofEnableDepthTest();
		ofPopMatrix();
	}
	else
	{
		ofBackgroundGradient(ofColor(30), ofColor(0));
		ofBackground(ofColor::black);
	}
	
	theCam->begin();
	ofPushMatrix();

	if (bWireframe) {                    // wireframe mode  (include axis)
		ofDisableLighting();
		ofSetColor(ofColor::slateGray);
		terrain.drawWireframe();
		if (bLanderLoaded) {
			lander.drawWireframe();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));
	}
	else {
		ofEnableLighting();              // shaded mode
		terrain.drawFaces();

		if (bLanderLoaded) {
			lander.drawFaces();
			if (!bTerrainSelected) drawAxis(lander.getPosition());
		}
		if (bTerrainSelected) drawAxis(ofVec3f(0, 0, 0));

		
	}


	if (bDisplayPoints) {                // display points as an option    
		glPointSize(3);
		ofSetColor(ofColor::green);
		terrain.drawVertices();
	}

	// highlight selected point (draw sphere around selected point)
	//
	if (bPointSelected) {
		ofSetColor(ofColor::yellow);
		ofDrawSphere(selectedPoint, .1);
	}

	ofNoFill();
	ofSetColor(ofColor::white);
	
	
		// debug - check first node to make sure bbox is correct
		//
	/*kdtree.drawLeafNodes(kdtree.root);
	kdtree.draw(kdtree.root, levels, 1);
	*/
	//////////////
	emitter.sys->draw();
	engineEmitter.sys->draw();
	engineEmitter2.sys -> draw();
	
	glDepthMask(GL_FALSE);

	ofSetColor(particleColor);

	ofEnableBlendMode(OF_BLENDMODE_ADD);
	ofEnablePointSprites();

	// begin drawing the particles
	shader.begin();

	particleTex.bind();
	vbo.draw(GL_POINTS, 0, (int)(engineEmitter.sys->particles.size() + engineEmitter2.sys->particles.size()));
	particleTex.unbind();

	//  end drawing in the camera
	shader.end();
	ofDisablePointSprites();
	ofDisableBlendMode();
	ofEnableAlphaBlending();

	// set back the depth mask
	//
	glDepthMask(GL_TRUE);

	//Change color back to normal
	ofSetColor(50, 50, 50);


	
	ofNoFill();
	ofSetColor(ofColor::white);
	//draw bounding boxes for lander
	drawBox(boundingBox);
	Box b2 = Box(boundingBox.parameters[0] + Vector3(50, 0, 0), boundingBox.parameters[1] + Vector3(50, 0, 0));
	drawBox(b2);
	Box b3 = Box(boundingBox.parameters[0] + Vector3(-50, 0, 0), boundingBox.parameters[1] + Vector3(-50, 0, 0));
	drawBox(b3);
	
	keyLight.draw();
	rimLight.draw();
	fillLight.draw();
	ofPopMatrix();
	ofDisableDepthTest();
	theCam->end();

	//Messages - Jessica Hoang
	gui.draw();
	string str = "Frame Rate: " + std::to_string(ofGetFrameRate());
	ofSetColor(ofColor::white);
	ofDrawBitmapString(str, ofGetWindowWidth() - 200, 20);

	string alt = "Altitude :" + std::to_string(lander.getPosition().y);
	ofSetColor(ofColor::white);
	ofDrawBitmapString(alt, ofGetWindowWidth()- 205, 30);

	string gas = "Gasoline: " + std::to_string(gasoline);
	ofSetColor(ofColor::white);
	ofDrawBitmapString(gas, ofGetWindowWidth() - 225, 40);

	string score1 = "Score: " + std::to_string(score);
	ofSetColor(ofColor::white);
	ofDrawBitmapString(score1, ofGetWindowWidth() - 225, 50);

	if (showLostMessage)
	{
		string lose = "Game Over";
		ofSetColor(ofColor::white);
		ofDrawBitmapString(lose, ofGetWindowWidth()/2, ofGetWindowHeight() / 2);

		string score2 = "Final Score: " + std::to_string(score);
		ofSetColor(ofColor::white);
		ofDrawBitmapString(score2, ofGetWindowWidth() / 2, ofGetWindowHeight() / 2 + 20);

	}

	if (showWinMessage)
	{
		string winner = "Mission complete! Try traveling to a different spot.";
		ofSetColor(ofColor::white);
		ofDrawBitmapString(winner, ofGetWindowWidth() / 2, ofGetWindowHeight() / 2);

		string again = "Press p to play again: ";
		ofSetColor(ofColor::white);
		ofDrawBitmapString(again, ofGetWindowWidth() / 2, ofGetWindowHeight() / 2 + 20);
	}

	if (!bGameActive && !win)
	{
		string title = "Press p to play";
		ofSetColor(ofColor::white);
		ofDrawBitmapString(title, ofGetWindowWidth() / 2, ofGetWindowHeight() / 2);
	}
}

// 

// Draw an XYZ axis in RGB at world (0,0,0) for reference.
//
void ofApp::drawAxis(ofVec3f location) {

	ofPushMatrix();
	ofTranslate(location);

	ofSetLineWidth(1.0);

	// X Axis
	ofSetColor(ofColor(255, 0, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(1, 0, 0));


	// Y Axis
	ofSetColor(ofColor(0, 255, 0));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 1, 0));

	// Z Axis
	ofSetColor(ofColor(0, 0, 255));
	ofDrawLine(ofPoint(0, 0, 0), ofPoint(0, 0, 1));

	ofPopMatrix();
}

//Check if a collision occured between a box and lander Justin
void ofApp:: checkCollision()
{
	Vector3 center = landerBox.center();
	pointOfContact = ofVec3f(center.x(), center.y() - landerBox.height() / 2, center.z()) + lander.getPosition();
	ofVec3f v = emitter.sys->particles[0].velocity;

	if (v.y >= 0) return;
	cout << "Checking collision..." << ofGetElapsedTimeMillis() << endl;
	TreeNode node;
	octree.pointIntersect(pointOfContact, octree.root, node);
	if (node.points.size() > 0 || lander.getPosition().y <= 6)
	{
		bCollision = true;
		cout << "Lander has intersected" << ofGetElapsedTimeMillis() << endl;
		impulseForce->apply(-60 * v * (restitution + 1));
		score = score + 2;
		if (v.y <= 0 && v.y >= -.1)
		{
			gravityForce->set(ofVec3f(0, 0, 0));
			emitter.sys->particles[0].velocity.set(0, 0, 0);
			cout << "It is in" << endl;
			lander.setPosition(0, 150, 0);
			emitter.sys->reset();
			engineEmitter.sys->reset();
			engineEmitter.start();
			engineEmitter2.sys->reset();
			engineEmitter2.start();
			
			gasoline += 200;
			score = score + 100;
			bCollision = false;
			win = true;
		}

		cout << "Gravity: " << gravityForce->gravity.y << "Velocity:" << v.y << endl;
		turbForce->set(ofVec3f(0, 0, 0), ofVec3f(0, 0, 0)); //Stop ship movement
	}

	
}

//check for front collision Justin
void ofApp::checkFrontCollision()
{
	Vector3 center = landerBox.center();
	frontPointOfContact = ofVec3f(center.x() - landerBox.length() / 2, center.y(), center.z()) + lander.getPosition();
	ofVec3f v = emitter.sys->particles[0].velocity;

	cout << "Checking the front of lander..." << ofGetElapsedTimeMillis() << endl;
	TreeNode node;
	octree.pointIntersect(frontPointOfContact, octree.root, node);
	if (node.points.size() > 0)
	{
		impulseForce->apply(-60 * v * (restitution + 1));
	}

}

//check for back collision Justin
void ofApp::checkBackCollision()
{
	Vector3 center = landerBox.center();
	backPointOfContact = ofVec3f(center.x() + landerBox.length() / 2, center.y(), center.z()) + lander.getPosition();
	ofVec3f v = emitter.sys->particles[0].velocity;

	cout << "Checking behind the lander..." << ofGetElapsedTimeMillis() << endl;

	TreeNode node;
	octree.pointIntersect(backPointOfContact, octree.root, node);
	if (node.points.size() > 0)
	{
		impulseForce->apply(-60 * v * (restitution + 1));

	}
	
}

//check for right collision Justin
void ofApp::checkRightCollision()
{
	Vector3 center = landerBox.center();
	rightPointOfContact = ofVec3f(center.x(), center.y(), center.z() + landerBox.width() / 2) + lander.getPosition();
	ofVec3f v = emitter.sys->particles[0].velocity;
	cout << "Checking the right of the lander..." << ofGetElapsedTimeMillis() << endl;
	TreeNode node;
	octree.pointIntersect(rightPointOfContact, octree.root, node);
	if (node.points.size() > 0)
	{
		impulseForce->apply(-60 * v * (restitution + 1));

	}

}

//Check for left collision of lander Justin
void ofApp::checkLeftCollision()
{
	Vector3 center = landerBox.center();
	leftPointOfContact = ofVec3f(center.x(), center.y(), center.z() - landerBox.width() / 2) + lander.getPosition();
	ofVec3f v = emitter.sys->particles[0].velocity;
	cout << "Checking the left of the lander..." << ofGetElapsedTimeMillis() << endl;
	TreeNode node;
	octree.pointIntersect(leftPointOfContact, octree.root, node);
	if (node.points.size() > 0)
	{
		impulseForce->apply(-60 * v * (restitution + 1));

	}
	

}

//depending on key pressed will result in an action - Jessica Hoang
void ofApp::keyPressed(int key) {

	switch (key) {
	case 'd':     // rotate spacecraft clockwise (about Y (UP) axis)
		temp += 10;
		lander.setRotation(0, temp, 0, 1, 0);
		break;
	case 'a':     // rotate spacecraft counter-clockwise (about Y (UP) axis)
		temp -= 10;
		lander.setRotation(0, temp, 0, 1, 0);
		break;
	case 'p':
		if (!bGameActive && !win && !showLostMessage)
		{
			bGameActive = true;
		}
		else if (!bGameActive && win)
		{
			win = false;
			showWinMessage = false;
			bGameActive = true;
			
		}
		break;

	case 'C':
	case 'c':
		if (cam.getMouseInputEnabled()) cam.disableMouseInput();
		else cam.enableMouseInput();
		break;
	case 'F':
	case 'f':
		ofToggleFullscreen();
		break;
	case 'H':
	case 'h':
		break;
	case 'r':
		cam.reset();
		break;
	case 's':
		savePicture();
		break;
	case 't':
		setCameraTarget();
		break;
	case 'u':
		break;
	case 'v':
		togglePointsDisplay();
		break;
	case 'V':
		break;
	case 'w':
		toggleWireframeMode();
		break;
	case OF_KEY_ALT:
		cam.enableMouseInput();
		bAltKeyDown = true;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = true;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_DEL:
		break;
	case 'j':
		theCam = &cam;
		break;
	case 'k':
		theCam = &frontCam;
	case 'l':
		theCam = &downCam;
	case 'm':
		theCam = &trackingCam;
		break;
	case OF_KEY_UP:
		playEngineSound();
		bSoundPlaying = true;
		bCollision = false;
		emitter.sys->reset();
		impulseForce->set(ofVec3f(0, 0, 0)); //Reset impulse
		turbForce->set(ofVec3f(-2, -2, -2), ofVec3f(2, 2, 2)); 
		if (bCtrlKeyDown) {
			thrustForceLunar->set(ofVec3f(0, 0, 3));
		}
		else {
			thrustForceLunar->set(ofVec3f(0, 3, 0));
		}
		engineEmitter.sys->reset();
		engineEmitter.start();
		engineEmitter2.sys->reset();
		engineEmitter2.start();
		gasoline--;
		break;
	case OF_KEY_DOWN:
		if (!bCollision) {
			playEngineSound();
			bSoundPlaying = true;
			emitter.sys->reset();
			impulseForce->set(ofVec3f(0, 0, 0)); //Reset impulse
			turbForce->set(ofVec3f(-2, -2, -2), ofVec3f(2, 2, 2)); 
			if (bCtrlKeyDown) {
				thrustForceLunar->set(ofVec3f(0, 0, -3));
			}
			else {
				thrustForceLunar->set(ofVec3f(0, -3, 0));
			}
			engineEmitter.sys->reset();
			engineEmitter.start();
			engineEmitter2.sys->reset();
			engineEmitter2.start();
			gasoline--;
		}
		break;
	case OF_KEY_LEFT:
		playEngineSound();
		bSoundPlaying = true;
		emitter.sys->reset();
		impulseForce->set(ofVec3f(0, 0, 0)); //Reset impulse
		thrustForceLunar->set(ofVec3f(-3, 0, 0));
		if (!bCollision) { turbForce->set(ofVec3f(-2, -2, -2), ofVec3f(2, 2, 2)); }
		engineEmitter.sys->reset();
		engineEmitter.start();
		engineEmitter2.sys->reset();
		engineEmitter2.start();
		gasoline--;
		break;
	case OF_KEY_RIGHT:
		playEngineSound();
		bSoundPlaying = true;
		emitter.sys->reset();
		impulseForce->set(ofVec3f(0, 0, 0)); //Reset impulse
		thrustForceLunar->set(ofVec3f(3, 0, 0));
		if (!bCollision) { turbForce->set(ofVec3f(-2, -2, -2), ofVec3f(2, 2, 2)); }
		engineEmitter.sys->reset();
		engineEmitter.start();
		engineEmitter2.sys->reset();
		engineEmitter2.start();
		gasoline--;
		break;
	default:
		break;
	}
}

void ofApp::toggleWireframeMode() {
	bWireframe = !bWireframe;
}



void ofApp::toggleSelectTerrain() {
	bTerrainSelected = !bTerrainSelected;
}

void ofApp::togglePointsDisplay() {
	bDisplayPoints = !bDisplayPoints;
}

void ofApp::keyReleased(int key) {

	switch (key) {

	case OF_KEY_ALT:
		cam.disableMouseInput();
		bAltKeyDown = false;
		break;
	case OF_KEY_CONTROL:
		bCtrlKeyDown = false;
		break;
	case OF_KEY_SHIFT:
		break;
	case OF_KEY_UP:
	case OF_KEY_DOWN:
	case OF_KEY_LEFT:
	case OF_KEY_RIGHT:
		engineSound.stop();
		bSoundPlaying = false;
		thrustForceLunar->set(ofVec3f(0, 0, 0));
		break;
	default:
		break;

	}
}



//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

	

}


//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

	//reset the  bool every time clicked
	bPointSelected = false;

	ofVec3f mouse(mouseX, mouseY);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	//Gives a ray
	Ray ray = Ray(Vector3(rayPoint.x, rayPoint.y, rayPoint.z),
		Vector3(rayDir.x, rayDir.y, rayDir.z));


	//Start the timer to measure the time for selection
	int startTime = ofGetElapsedTimeMillis();

	//If the ray intersects with a leaf node, set the selected point to the first point in that node
	octree.intersect(ray, octree.root, selectedNode);
	if (selectedNode.points.size() > 0) {
		bPointSelected = true;
		selectedPoint = terrain.getMesh(0).getVertex(selectedNode.points[0]);
	}
	cout << "Select Time: " << ofGetElapsedTimeMillis() - startTime << endl;


}

//Justin
void ofApp::playEngineSound() {
	if (bSoundPlaying) return;
	if (!engineSound.isPlaying()) engineSound.play();
}

//draw a box from a "Box" class  
//
void ofApp::drawBox(const Box &box) {
	Vector3 min = box.parameters[0];
	Vector3 max = box.parameters[1];
	Vector3 size = max - min;
	Vector3 center = size / 2 + min;
	ofVec3f p = ofVec3f(center.x(), center.y(), center.z());
	float w = size.x();
	float h = size.y();
	float d = size.z();
	ofDrawBox(p, w, h, d);
}

// return a Mesh Bounding Box for the entire Mesh
//
Box ofApp::meshBounds(const ofMesh & mesh) {
	int n = mesh.getNumVertices();
	ofVec3f v = mesh.getVertex(0);
	ofVec3f max = v;
	ofVec3f min = v;
	for (int i = 1; i < n; i++) {
		ofVec3f v = mesh.getVertex(i);

		if (v.x > max.x) max.x = v.x;
		else if (v.x < min.x) min.x = v.x;

		if (v.y > max.y) max.y = v.y;
		else if (v.y < min.y) min.y = v.y;

		if (v.z > max.z) max.z = v.z;
		else if (v.z < min.z) min.z = v.z;
	}
	return Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
}



//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

	// 
	//  implement your code here to drag the lander around

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
	
}

bool ofApp::doPointSelection() {

	ofMesh mesh = terrain.getMesh(0);
	int n = mesh.getNumVertices();
	float nearestDistance = 0;
	int nearestIndex = 0;

	bPointSelected = false;

	ofVec2f mouse(mouseX, mouseY);
	vector<ofVec3f> selection;

	// We check through the mesh vertices to see which ones
	// are "close" to the mouse point in screen space.  If we find
	// points that are close, we store them in a vector (dynamic array)
	//
	for (int i = 0; i < n; i++) {
		ofVec3f vert = mesh.getVertex(i);
		ofVec3f posScreen = cam.worldToScreen(vert);
		float distance = posScreen.distance(mouse);
		if (distance < selectionRange) {
			selection.push_back(vert);
			bPointSelected = true;
		}
	}

	//  if we found selected points, we need to determine which
	//  one is closest to the eye (camera). That one is our selected target.
	//
	if (bPointSelected) {
		float distance = 0;
		for (int i = 0; i < selection.size(); i++) {
			ofVec3f point = cam.worldToCamera(selection[i]);

			// In camera space, the camera is at (0,0,0), so distance from
			// the camera is simply the length of the point vector
			//
			float curDist = point.length();

			if (i == 0 || curDist < distance) {
				distance = curDist;
				selectedPoint = selection[i];
			}
		}
	}
	return bPointSelected;
}

// Set the camera to use the selected point as it's new target
//  
void ofApp::setCameraTarget() {

}


//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}



//--------------------------------------------------------------
// setup basic ambient lighting in GL  (for now, enable just 1 light)
//
void ofApp::initLightingAndMaterials() {

	//setting up lights Justin
	keyLight.setup();
	keyLight.enable();
	keyLight.setAreaLight(2, 5);
	keyLight.setAmbientColor(ofColor(150, 250, 250));
	keyLight.setDiffuseColor(ofColor(200, 250, 250));
	keyLight.setSpecularColor(220);

	keyLight.rotate(-90, ofVec3f(1, 0, 0));
	keyLight.rotate(-90, ofVec3f(0, 1, 0));
	keyLight.setPosition(30, 200, 0);

	fillLight.setup();
	fillLight.enable();
	fillLight.setSpotlight();
	fillLight.setScale(.05);
	fillLight.setSpotlightCutOff(15);
	fillLight.setAttenuation(2, .001, .001);
	fillLight.setAmbientColor(ofFloatColor(150, 250, 250));
	fillLight.setDiffuseColor(ofFloatColor(150, 250, 250));
	fillLight.setSpecularColor(ofFloatColor(150, 250, 250));
	fillLight.rotate(-10, ofVec3f(1, 0, 0));
	fillLight.rotate(-45, ofVec3f(0, 1, 0));
	fillLight.setPosition(30, 200, 0);

	rimLight.setup();
	rimLight.enable();
	rimLight.setSpotlight();
	rimLight.setScale(.05);
	rimLight.setSpotlightCutOff(30);
	rimLight.setAttenuation(.2, .001, .001);
	rimLight.setAmbientColor(ofFloatColor(150, 250, 250));
	rimLight.setDiffuseColor(ofFloatColor(150, 250, 250));
	rimLight.setSpecularColor(ofFloatColor(150, 250, 250));
	rimLight.rotate(180, ofVec3f(0, 1, 0));
	rimLight.setPosition(30, 200, 0);
	static float ambient[] =
	{ .5f, .5f, .5, 1.0f };
	static float diffuse[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float position[] =
	{ 5.0, 5.0, 5.0, 0.0 };

	static float lmodel_ambient[] =
	{ 1.0f, 1.0f, 1.0f, 1.0f };

	static float lmodel_twoside[] =
	{ GL_TRUE };


	glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, position);

	glLightfv(GL_LIGHT1, GL_AMBIENT, ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, position);


	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glLightModelfv(GL_LIGHT_MODEL_TWO_SIDE, lmodel_twoside);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//	glEnable(GL_LIGHT1);
	glShadeModel(GL_SMOOTH);
}

void ofApp::savePicture() {
	ofImage picture;
	picture.grabScreen(0, 0, ofGetWidth(), ofGetHeight());
	picture.save("screenshot.png");
	cout << "picture saved" << endl;
}

//--------------------------------------------------------------
//
// support drag-and-drop of model (.obj) file loading.  when
// model is dropped in viewport, place origin under cursor
//
void ofApp::dragEvent(ofDragInfo dragInfo) {
	if (lander.loadModel(dragInfo.files[0])) {
		bLanderLoaded = true;
		lander.setScaleNormalization(false);
		//	lander.setScale(.5, .5, .5);
		lander.setPosition(0, 0, 0);
		//		lander.setRotation(1, 180, 1, 0, 0);

				// We want to drag and drop a 3D object in space so that the model appears 
				// under the mouse pointer where you drop it !
				//
				// Our strategy: intersect a plane parallel to the camera plane where the mouse drops the model
				// once we find the point of intersection, we can position the lander/lander
				// at that location.
				//

				// Setup our rays
				//
		glm::vec3 origin = theCam->getPosition();
		glm::vec3 camAxis = theCam->getZAxis();
		glm::vec3 mouseWorld = theCam->screenToWorld(glm::vec3(mouseX, mouseY, 0));
		glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
		float distance;

		bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);
		if (hit) {
			// find the point of intersection on the plane using the distance 
			// We use the parameteric line or vector representation of a line to compute
			//
			// p' = p + s * dir;
			//
			glm::vec3 intersectPoint = origin + distance * mouseDir;

			// Now position the lander's origin at that intersection point
			//
			glm::vec3 min = lander.getSceneMin();
			glm::vec3 max = lander.getSceneMax();
			float offset = (max.y - min.y) / 2.0;
			lander.setPosition(intersectPoint.x, intersectPoint.y - offset, intersectPoint.z);

			// set up bounding box for lander while we are at it
			//
			//landerBounds = Box(Vector3(min.x, min.y, min.z), Vector3(max.x, max.y, max.z));
		}
	}


}


//  intersect the mouse ray with the plane normal to the camera 
//  return intersection point.   (package code above into function)
//
//glm::vec3 ofApp::getMousePointOnPlane() {
//	// Setup our rays
//	//
//	glm::vec3 origin = theCam->getPosition();
//	glm::vec3 camAxis = theCam->getZAxis();
//	glm::vec3 mouseWorld = theCam->screenToWorld(glm::vec3(mouseX, mouseY, 0));
//	glm::vec3 mouseDir = glm::normalize(mouseWorld - origin);
//	float distance;
//
//	bool hit = glm::intersectRayPlane(origin, mouseDir, glm::vec3(0, 0, 0), camAxis, distance);
//
//	if (hit) {
//		// find the point of intersection on the plane using the distance 
//		// We use the parameteric line or vector representation of a line to compute
//		//
//		// p' = p + s * dir;
//		//
//		glm::vec3 intersectPoint = origin + distance * mouseDir;
//
//		return intersectPoint;
//	}
//	else return glm::vec3(0, 0, 0);
//}
bool ofApp::mouseIntersectPlane(ofVec3f planePoint, ofVec3f planeNorm, ofVec3f &point) {
	glm::vec3 mouse(mouseX, mouseY, 0);
	ofVec3f rayPoint = cam.screenToWorld(mouse);
	ofVec3f rayDir = rayPoint - cam.getPosition();
	rayDir.normalize();
	return (rayIntersectPlane(rayPoint, rayDir, planePoint, planeNorm, point));
}
