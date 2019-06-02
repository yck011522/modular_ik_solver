#ifdef WIN32
#define NOMINMAX
#endif
#define _USE_MATH_DEFINES
#include <math.h>

#include <app.h>

#include <NewtonFunctionMinimizer.h>
#include <RigidBodySimulation.h>
#include <MechanismOptimizer.h>

#include "mechanisms.h"

#include <iostream>
#include <chrono>
#include <algorithm>

#include <Eigen/Core>

using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::VectorXd;

#define PLOT_N 100 // number of data points in plot

class MoveRigidBodyObj : public ObjectiveFunction
{
public:
    MoveRigidBodyObj(Vector2d pTarget, Vector2d pLocal)
        : rb({0, 1.0}), pTarget(pTarget), pLocal(pLocal) {
    }

    double evaluate(const VectorXd& x) const override {
        return (pTarget - rb.pWorld(x, pLocal)).squaredNorm();
    }

public:
    RigidBody rb;
    Vector2d pTarget, pLocal;
};

class RigidBodyApp : public App
{
public:
	RigidBodyApp(int width, int height, const char * title, float pixelRatio = 0.f)
		: App(width, height, title, pixelRatio), base(width) {

		clear_color = ImVec4(0.8f, 0.8f, 0.8f, 1.00f);
		lastFrame = std::chrono::high_resolution_clock::now();

		font = nvgCreateFont(vg, "sans", DATA_FOLDER"/Roboto-Regular.ttf");
		if (font == -1) {
			printf("Could not add font.\n");
		}

		for (float & d : dataEnergy)
			d = 0;

		sim = make4barSim();
		mechOpt = MechanismOptimizer(sim);
	}

    void process() override{
		// move image if right mouse button is pressed
		if(mouseDown[GLFW_MOUSE_BUTTON_RIGHT]){
			auto dw = (int)(cursorPos[0] - cursorPosDown[0]);
			auto dh = (int)(cursorPos[1] - cursorPosDown[1]);
			translation[0] += dw/(double)base;
			translation[1] -= dh/(double)base;
			cursorPosDown[0] = cursorPos[0];
			cursorPosDown[1] = cursorPos[1];
		}

		// run at 60fps, or in slow mo
		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
		if(std::chrono::duration_cast<std::chrono::milliseconds>(now-lastFrame).count() > ((slowMo) ? 320 : 16)){

			if(selectedRb != -1){
				double scale = 1.0;
				if(keyDown[GLFW_KEY_UP])
					scale = 1.01;
				else if(keyDown[GLFW_KEY_DOWN])
					scale = 0.99;
				if(scale != 1.0){
					sim.scaleRigidBody(selectedRb, scale);

					if(sim.motorIdx != -1)
                        trackedTrajectory = sim.recordTrajectory();
						cout << "trackedTrajectory: " << trackedTrajectory << endl;
				}
			}

			if(sim.motorIdx != -1){
				if(keyDown[GLFW_KEY_LEFT])
					sim.fixedAngle()[sim.motorIdx].angle += 4 * M_PI / 180;
				if(keyDown[GLFW_KEY_RIGHT])
					sim.fixedAngle()[sim.motorIdx].angle -= 4 * M_PI / 180;

				if(sim.fixedAngle()[sim.motorIdx].angle > 2*M_PI)
					sim.fixedAngle()[sim.motorIdx].angle -= 2*M_PI;
				if(sim.fixedAngle()[sim.motorIdx].angle < 0)
					sim.fixedAngle()[sim.motorIdx].angle += 2*M_PI;
			}


			if(mouseDown[GLFW_MOUSE_BUTTON_LEFT] && selectedRb != -1){

                Vector2d pos = fromScreen(cursorPos[0], cursorPos[1]);
                const auto &rb = sim.rigidbodies()[selectedRb];
                MoveRigidBodyObj obj({pos, selectedRbLocal});
                GradientDescentVariableStep gd;
                VectorXd xRb = sim.x.segment<3>(rb.dofIdx);
                gd.minimize(&obj, xRb, false);
                sim.x.segment<3>(rb.dofIdx) = xRb;
			}

			if(runSim){
				sim.run();
			}

			energy = sim.energy.evaluate(sim.x);
			dataCounter %= PLOT_N;
			dataEnergy.at(dataCounter++) = (float)energy;

			lastFrame = now;
		}
	}

    void drawScene() override {

		// this is the GUI
		{
			ImGui::Begin("Assignement 1");

			ImGui::TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "left mouse:  Select rigid bodies, apply forces");
			ImGui::TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "right mouse: move around");
			ImGui::TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "mouse wheel: zoom");
			ImGui::TextColored(ImVec4(1.f, 1.f, 1.f, 0.5f), "space bar:   play/pause");

			//Define the mechanisms to include in main app.
			const RigidBodySimulation sims[] = {
				makeJansenSim(),
				make4barSim(),
				makePrismaticSim1(),
				makePrismaticSim2(),
				makePrismaticSim3(),
				makePrismaticSim4_XY(),
				makePrismaticSim5_XY(),
				makePrismaticSim6_XYLoop(),
				makeTangentSim1(),
				makeTangentSim2(),
			};
			const int numberOfSims = 10;

			//Construct the dropdown list based on the included function
			char* items[numberOfSims];
			for (int i = 0; i < numberOfSims; i++) {
				items[i] = sims[i].name;
			}
			
			if(ImGui::Combo("Mechanism", &loadMech, items, numberOfSims)){
				sim = sims[loadMech];
				targetTrajectory.resize(0, 2);
				//if (loadMech == 0) {
				//	targetTrajectory = makeJansenTargetPath();
				//	sim.energy.fixedAngleEnabled = true;
				//	trackedTrajectory = sim.recordTrajectory();
				//}
				mechOpt = MechanismOptimizer(sim);
			}

            if(ImGui::CollapsingHeader("Simulation")){
                ImGui::Checkbox("run simulation", &runSim);
                ImGui::PlotLines("Energy", dataEnergy.data(), PLOT_N, dataCounter, "energy", 0, 1.0, ImVec2(0, 100));
                ImGui::Separator();


                if(sim.motorIdx != -1){
                    ImGui::Checkbox("Fixed Angles enabled", &sim.energy.fixedAngleEnabled);
                    auto &fixedAngle = sim.fixedAngle()[sim.motorIdx];
                    float angle = (sim.energy.fixedAngleEnabled) ?
                                fixedAngle.angle :
                                sim.rigidbodies()[fixedAngle.rbIdx].theta(sim.x);
                    if(ImGui::SliderAngle("angle", &angle, 0, 360)){
                        fixedAngle.angle = angle;
                        sim.rigidbodies()[fixedAngle.rbIdx].theta(sim.x) = angle;
                    }
                    ImGui::Separator();
                }
            }

   //         if(sim.motorIdx != -1 && ImGui::CollapsingHeader("Design Optimization")){

			//	ImGui::Checkbox("Run optimization", &isMechOpt);
			//	if(isMechOpt){
   //                 mechOpt.targetPath = targetTrajectory;
   //                 mechOpt.optimizeTrajectory();
			//		VectorXd p = sim.getDesignParameters();
			//		if((p-mechOpt.p).norm() > 1e-10){
			//			sim.setDesignParameters(mechOpt.p);
   //                     trackedTrajectory = sim.recordTrajectory();
			//		}
			//	}

   //             if(ImGui::Button("print link lengths"))
   //                 std::cout << "Link lengths:" << std::endl <<
   //                              sim.getDesignParameters() << std::endl;
			//}

			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::End();
		}

		for (const auto &f : sim.fixedAngle()) {
			nvgResetTransform(vg);
			nvgBeginPath(vg);
			const auto &rb = sim.rigidbodies()[f.rbIdx];
			Vector2d p = rb.pos(sim.x);

			nvgCircle(vg, toScreen(p.x(), 0), toScreen(p.y(), 1), toScreen(0.5));
			nvgFillColor(vg, (sim.motorIdx == -1) ? nvgRGBAf(0, 0, 1, 0.5) : nvgRGBAf(0, 1, 1, 0.5));
			nvgFill(vg);
		}

		int i = 0;
		for (const auto &rb : sim.rigidbodies()) {
			nvgResetTransform(vg);
			Vector2d p = rb.pos(sim.x);
			nvgTranslate(vg, toScreen(p.x(), 0), toScreen(p.y(), 1));
			nvgRotate(vg, -rb.theta(sim.x));
			nvgBeginPath(vg);
			double r = rb.width/2;
			nvgRoundedRect(vg, toScreen(-rb.length*0.5 - r), toScreen(-rb.width*0.5), toScreen(rb.length + 2*r), toScreen(rb.width), toScreen(r));
			if(i == selectedRb)
				nvgFillColor(vg, nvgRGBAf(1.0, 0.9, 0.7, 0.5));
			else
				nvgFillColor(vg, nvgRGBAf(0.5, 0.5, 0.5, 0.5));
			nvgFill(vg);
			nvgStrokeColor(vg, nvgRGBAf(0, 0, 0, 1));
			nvgStrokeWidth(vg, 2*pixelRatio);
			nvgStroke(vg);

			nvgFontSize(vg, toScreen(0.3));
			nvgFontFace(vg, "sans");

			nvgFillColor(vg, nvgRGBAf(0,0,0,1));
			nvgText(vg, 0, 100/zoom, rb.name.c_str(), nullptr);
			i++;
		}

		for (const auto &joint : sim.hingeJoints()) {
			nvgResetTransform(vg);
			nvgBeginPath(vg);
			const auto &rb0 = sim.rigidbodies()[joint.rbIdx[0]];
			const auto &rb1 = sim.rigidbodies()[joint.rbIdx[1]];
			Vector2d p0 = rb0.pWorld(sim.x, joint.local[0]);
			Vector2d p1 = rb1.pWorld(sim.x, joint.local[1]);
			nvgMoveTo(vg, toScreen(p0.x(), 0), toScreen(p0.y(), 1));
			nvgLineTo(vg, toScreen(p1.x(), 0), toScreen(p1.y(), 1));
			nvgStrokeColor(vg, nvgRGBAf(1, 0, 0, 0.5));
			nvgStrokeWidth(vg, 2.0*pixelRatio);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, toScreen(p0.x(), 0), toScreen(p0.y(), 1), toScreen(rb0.width/3));
			nvgCircle(vg, toScreen(p1.x(), 0), toScreen(p1.y(), 1), toScreen(rb1.width/3));
			nvgStrokeColor(vg, nvgRGBAf(1, 0, 0, 0.5));
			nvgStrokeWidth(vg, 2.0*pixelRatio);
			nvgStroke(vg);
		}

		//Added Draw Point On Line Joint Graphics
		for (const auto &joint : sim.pointOnLineJoints()) {
			nvgResetTransform(vg);

			nvgBeginPath(vg);
			const auto &rb0 = sim.rigidbodies()[joint.rbIdx[0]];

			Vector2d p0_bgn = rb0.pWorld(sim.x, joint.local0Point);
			Vector2d p0_end = rb0.pWorld(sim.x, joint.local0Point + joint.local0Vector);
			nvgMoveTo(vg, toScreen(p0_bgn.x(), 0), toScreen(p0_bgn.y(), 1));
			nvgLineTo(vg, toScreen(p0_end.x(), 0), toScreen(p0_end.y(), 1));
			nvgStrokeColor(vg, nvgRGBAf(0, 0, 1, 0.5));
			nvgStrokeWidth(vg, 2.0*pixelRatio);
			nvgStroke(vg);
			
			//Draw circle on rb1 point
			const auto &rb1 = sim.rigidbodies()[joint.rbIdx[1]];
			Vector2d p1 = rb1.pWorld(sim.x, joint.local1Point);

			nvgBeginPath(vg);
			nvgCircle(vg, toScreen(p1.x(), 0), toScreen(p1.y(), 1), toScreen(rb1.width / 3));
			nvgStrokeColor(vg, nvgRGBAf(0, 0, 1, 0.5));
			nvgStrokeWidth(vg, 2.0*pixelRatio);
			nvgStroke(vg);
		}

		for (const auto &f : sim.fixed()) {
			nvgResetTransform(vg);
			nvgBeginPath(vg);
			const auto &rb = sim.rigidbodies()[f.rbIdx];
			Vector2d p0 = rb.pWorld(sim.x, f.localPos);
			Vector2d p1 = f.pos;
			nvgMoveTo(vg, toScreen(p0.x(), 0), toScreen(p0.y(), 1));
			nvgLineTo(vg, toScreen(p1.x(), 0), toScreen(p1.y(), 1));
			nvgStrokeColor(vg, nvgRGBAf(0.2, 0.2, 0, 0.5));
			nvgStrokeWidth(vg, 2.0*pixelRatio);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, toScreen(p0.x(), 0), toScreen(p0.y(), 1), toScreen(rb.width/4));
			nvgCircle(vg, toScreen(p1.x(), 0), toScreen(p1.y(), 1), toScreen(rb.width/4));
			nvgStrokeColor(vg, nvgRGBAf(0.2, 0.2, 0, 0.5));
			nvgStrokeWidth(vg, 2.0*pixelRatio);
			nvgStroke(vg);
		}

		// draw tracked point
		if(sim.trackRBPoint.rbIdx >= 0){
			nvgResetTransform(vg);
			nvgBeginPath(vg);
			const auto &rb = sim.rigidbodies()[sim.trackRBPoint.rbIdx];
			Vector2d p = rb.pWorld(sim.x, sim.trackRBPoint.local);
			nvgCircle(vg, toScreen(p.x(), 0), toScreen(p.y(), 1), 2.0);
			nvgFillColor(vg, nvgRGBAf(0.2, 0.8, 0.2, 0.5));
			nvgFill(vg);
		}

		auto draw_path = [=](const Matrix<double, -1, 2> &path, NVGcolor color){
			if(path.rows() > 0){
				nvgBeginPath(vg);

				nvgMoveTo(vg, toScreen(path(0, 0), 0), toScreen(path(0, 1), 1));
				for (int i = 0; i < path.rows(); i++) {
					nvgLineTo(vg, toScreen(path(i, 0), 0), toScreen(path(i, 1), 1));
				}
				nvgStrokeColor(vg, color);
				nvgStrokeWidth(vg, 2);
				nvgStroke(vg);

				nvgBeginPath(vg);
				for (int i = 0; i < path.rows(); i++)
					nvgCircle(vg, toScreen(path(i, 0), 0), toScreen(path(i, 1), 1), toScreen(0.07));
				nvgFillColor(vg, color);
				nvgFill(vg);
			}
		};


        draw_path(trackedTrajectory, nvgRGBAf(0.2, 0.8, 0.2, 0.5));
        draw_path(targetTrajectory, nvgRGBAf(0.8, 0.5, 0.2, 0.5));

	}

protected:
    void keyPressed(int key, int  /*mods*/) override {
		// play / pause with space bar
		if(key == GLFW_KEY_SPACE)
			runSim = !runSim;
	}

    void mousePressed(int button) override {
		cursorPosDown[0] = cursorPos[0];
		cursorPosDown[1] = cursorPos[1];

		if(button == GLFW_MOUSE_BUTTON_LEFT){
			Vector2d cursor = fromScreen(cursorPos[0], cursorPos[1]);

            int i = 0;
            selectedRb = -1;
            for (const auto &rb : sim.rigidbodies()) {
                auto rot = rotationMatrix(-rb.theta(sim.x));
                Vector2d d = rot * (cursor - rb.pos(sim.x));
                if(std::abs(d.x()) <= rb.length/2 && std::abs(d.y()) <= rb.width/2){
                    selectedRb = i;
                    selectedRbLocal = d;
                    break;
                }
                i++;
            }
		}
	}

    void mouseReleased(int  /*button*/) override {
	}

    void scrollWheel(double  /*xoffset*/, double yoffset) override {
		double zoomOld = zoom;
		zoom *= std::pow(1.10, yoffset);
		for (int dim = 0; dim < 2; ++dim) {
			double c = cursorPos[dim]/(double) ((dim == 0) ? base : -base);
			translation[dim] = c - zoomOld/zoom * (c-translation[dim]);
		}
	}

    void windowResized(int /*w*/, int /*h*/) override {
	}


private:

	VectorXd fromScreen(int i, int j, int w, int h) const {
		VectorXd x(2);
		x[0] = ((double)i/(double)w - translation[0])*zoom/pixelRatio;
		x[1] = (-(double)j/(double)h - translation[1])*zoom/pixelRatio;
		return x;
	}

	template<class S>
	VectorXd fromScreen(S i, S j) const {
		return fromScreen((double)i, (double)j, base, base);
	}

	double toScreen(double s, int dim) const {
		return (s/zoom*pixelRatio + translation[dim]) * (double)((dim == 0) ? base : -base);
	}

	double toScreen(double s) const {
		return s/zoom*pixelRatio * base;
	}

private:
	int font = -1;

	int loadMech = 0;
	bool runSim = false;
	std::chrono::high_resolution_clock::time_point lastFrame;
	bool slowMo = false;

	double cursorPosDown[2]{};
	double translation[2] = {0.75*pixelRatio, -0.25*pixelRatio};
	double zoom = 24;
	int base;

	int selectedRb = -1;
    Vector2d selectedRbLocal;

	bool isMechOpt = false;

public:
	// optimization
	double energy = 0;
	int dataCounter = 0;
	std::array<float, PLOT_N> dataEnergy{};

	RigidBodySimulation sim;

    Matrix<double, -1, 2> trackedTrajectory, targetTrajectory;
	MechanismOptimizer mechOpt;
};

int main(int, char**)
{
	// If you have high DPI screen settings, you can change the pixel ratio
	// accordingly. E.g. for 200% scaling use `pixelRatio = 2.f`
	RigidBodyApp app(1080, 720, "Assignement 1");

	app.run();
	return 0;
}
