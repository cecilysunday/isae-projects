// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about using paths for defining trajectories
//
// =============================================================================

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkTrajectory.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "ProjConfig.h"
#include "ProjCalcs.h"
#include "ProjWriteData.h"
#include "ProjProc.h"

#include <random>

// If IRRLICHT is enabled, add irrlicht headers and namespaces
#ifdef CHRONO_IRRLICHT
#include <irrlicht.h>
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create a Chrono::Engine physical system
    // ChSystemNSC sys;
    ChSystemMulticoreSMC sys;

    //
    // EXAMPLE 1:
    //

    // Create a ChBody that contains the trajectory (a floor, fixed body)

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(3, 0.2, 3, 1000, true, false);
    floor->SetBodyFixed(false);
    // floor->SetRot(Q_from_AngAxis(0.1,VECT_Z));
    sys.Add(floor);

    // auto body = std::shared_ptr<ChBody>(sys.NewBody());
	// body->SetIdentifier(1);
	// body->SetMass(1);
	// body->SetPos(ChVector<>(0));
	// body->SetRot(QUNIT);
	// body->SetBodyFixed(false);
	// body->SetCollide(true);

    // auto mat_pw = chrono_types::make_shared<ChMaterialSurfaceSMC>();
	// mat_pw->SetYoungModulus(1);
	// mat_pw->SetPoissonRatio(1);
	// mat_pw->SetSfriction(1);
	// mat_pw->SetKfriction(1);
	// mat_pw->SetRollingFriction(1);
	// mat_pw->SetSpinningFriction(1);
	// mat_pw->SetRestitution(1);
    // mat_pw->SetAdhesion(1);

	// body->GetCollisionModel()->ClearModel();
	// utils::AddBoxGeometry(body.get(), mat_pw, ChVector<>(3,0.2,3), VNULL, QUNIT, true);
	// body->GetCollisionModel()->BuildModel();

	// // Attach a color to the visible container
	// #ifdef CHRONO_IRRLICHT
	// auto mvisual = chrono_types::make_shared<ChBoxShape>();
	// mvisual->GetBoxGeometry().Size = ChVector<>(3,0.2,3);
	// mvisual->SetColor(ChColor(0.35f, 0.85f, 0.15f));
	// body->AddVisualShape(mvisual);
	// #endif

	// // Add the wall to the system
	// sys.AddBody(body);

    // auto floor = chrono_types::make_shared<ChBody>();
    // floor->SetBodyFixed(false);
 
    // // Contact material
    // auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
 
    // // Define a collision shape
    // floor->GetCollisionModel()->ClearModel();
    // floor->GetCollisionModel()->AddBox(floor_mat, 10, 0.5, 10, ChVector<>(0, -1, 0));
    // floor->GetCollisionModel()->BuildModel();
    // floor->SetCollide(true);
 
    // // Add body to system
    // sys.Add(floor);

    // // ==Asset== attach a 'box' shape.
    // // Note that assets are managed via shared pointer, so they can also be shared).
    // auto boxfloor = chrono_types::make_shared<ChBoxShape>();
    // boxfloor->GetBoxGeometry().Size = ChVector<>(10, 0.5, 10);
    // boxfloor->SetColor(ChColor(0.2f, 0.3f, 1.0f));
    // floor->AddVisualShape(boxfloor, ChFrame<>(ChVector<>(0, -1, 0), QUNIT));

    // Create a ChLinePath geometry, and insert sub-paths:
    auto path = chrono_types::make_shared<ChLinePath>();
    ChLineSegment mseg1(ChVector<>(1, 2, 0), ChVector<>(2, 2, 0));
    path->AddSubLine(mseg1);
    ChLineArc marc1(ChCoordsys<>(ChVector<>(2, 2.5, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2, true);
    path->AddSubLine(marc1);
    ChLineSegment mseg2(ChVector<>(2, 3, 0), ChVector<>(1, 3, 0));
    path->AddSubLine(mseg2);
    ChLineArc marc2(ChCoordsys<>(ChVector<>(1, 2.5, 0)), 0.5, CH_C_PI_2, -CH_C_PI_2, true);
    path->AddSubLine(marc2);
    path->Set_closed(true);

    // Create a ChLineShape, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    auto pathasset = chrono_types::make_shared<ChLineShape>();
    pathasset->SetLineGeometry(path);
    floor->AddVisualShape(pathasset);

    // Create a body that will follow the trajectory

    // auto pendulum = chrono_types::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000, false, false);
    // pendulum->SetPos(ChVector<>(1, 1.5, 0));
    // sys.Add(pendulum);

    // The trajectory constraint:

    // auto trajectory = chrono_types::make_shared<ChLinkTrajectory>();

    // // Define which parts are connected (the trajectory is considered in the 2nd body).
    // trajectory->Initialize(pendulum,               // body1 that follows the trajectory
    //                        floor,                  // body2 that 'owns' the trajectory
    //                        ChVector<>(0, 0.5, 0),  // point on body1 that will follow the trajectory
    //                        path                    // the trajectory (reuse the one already added to body2 as asset)
    // );

    // Optionally, set a function that gets the curvilinear
    // abscyssa s of the line, as a function of time s(t). By default it was simply  s=t.
    // auto mspacefx = chrono_types::make_shared<ChFunction_Ramp>(0, 0.5);
    // trajectory->Set_space_fx(mspacefx);

    // sys.Add(trajectory);

    //
    // EXAMPLE 2:
    //

    // Create a ChBody that contains the trajectory

    // auto wheel = chrono_types::make_shared<ChBody>();
    // std::cout<<"Avant declaration wheel \n";
    // auto wheel = std::shared_ptr<ChBody>(sys.NewBody());
    // wheel->SetPos(ChVector<>(-3, 2, 0));
   
    // // sys.Add(wheel);

    // // auto wheel = chrono_types::make_shared<ChBodyEasyBox>(3, 0.2, 3, 1000, true, false);
    // // wheel->SetBodyFixed(false);
    // // // floor->SetRot(Q_from_AngAxis(0.1,VECT_Z));
    // // sys.Add(wheel);

    // // Create a motor that spins the wheel
    // // auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    // // motor->Initialize(wheel, floor, ChFrame<>(ChVector<>(-3, 2, 0)));
    // // motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 4.0));
    // // sys.AddLink(motor);

    // Create a ChLinePath geometry, and insert sub-paths:
    // auto glyph = chrono_types::make_shared<ChLinePath>();
    // ChLineSegment ms1(ChVector<>(-0.5, -0.5, 0), ChVector<>(0.5, -0.5, 0));
    // glyph->AddSubLine(ms1);
    // ChLineArc ma1(ChCoordsys<>(ChVector<>(0.5, 0, 0)), 0.5, -CH_C_PI_2, CH_C_PI_2, true);
    // glyph->AddSubLine(ma1);
    // ChLineSegment ms2(ChVector<>(0.5, 0.5, 0), ChVector<>(-0.5, 0.5, 0));
    // glyph->AddSubLine(ms2);
    // ChLineArc ma2(ChCoordsys<>(ChVector<>(-0.5, 0, 0)), 0.5, CH_C_PI_2, -CH_C_PI_2, true);
    // glyph->AddSubLine(ma2);
    // glyph->SetPathDuration(1);
    // glyph->Set_closed(true);

    // Create a ChLineShape, a visualization asset for lines.
    // The ChLinePath is a special type of ChLine and it can be visualized.
    // #ifdef CHRONO_IRRLICHT
    // auto glyphasset = chrono_types::make_shared<ChLineShape>();
    // glyphasset->SetLineGeometry(glyph);
    // floor->AddVisualShape(glyphasset);
    // #endif
    // std::cout<<"Avant addbody \n";
    // sys.AddBody(wheel);
    // std::cout<<"Après addbody \n";
;
    // Create a body that will slide on the glyph

    // auto pendulum2 = chrono_types::make_shared<ChBodyEasyBox>(0.1, 1, 0.1, 1000, false, false);
    // pendulum2->SetPos(ChVector<>(-3, 1, 0));
    // sys.Add(pendulum2);

    // The glyph constraint:

    // auto glyphconstraint = chrono_types::make_shared<ChLinkPointSpline>();

    // // Define which parts are connected (the trajectory is considered in the 2nd body).
    // glyphconstraint->Initialize(pendulum2,  // body1 that follows the trajectory
    //                             wheel,      // body2 that 'owns' the trajectory
    //                             true,
    //                             ChCoordsys<>(ChVector<>(0, 0.5, 0)),  // point on body1 that will follow the trajectory
    //                             ChCoordsys<>());

    // glyphconstraint->Set_trajectory_line(glyph);

    // sys.Add(glyphconstraint);



    // Create the Irrlicht visualization system
    #ifdef CHRONO_IRRLICHT
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);

	vis->SetWindowSize(800, 600);
	vis->SetWindowTitle("BOX_SET");
    vis->SetCameraVertical(CameraVerticalDir::Z);
	vis->Initialize();
	vis->AddTypicalLights();
	vis->AddSkyBox();
	// vis->AddCamera(ChVector<>(0, 10 * 2, 10), ChVector<>(0, 0, 10));
	vis->AddCamera(ChVector<>(0, 4, -6));
    // vis->AddLight(ChVector<>(0, 10 * 2, 10), 10 * 2);
    #endif
    std::cout<<"Apres def irrlicht \n";
    // This means that contactforces will be shown in Irrlicht application
    // vis->SetSymbolScale(0.2);
    // vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);

    // Simulation loop
    double timestep = 0.01;
    // ChRealtimeStepTimer realtime_timer;

    // while (vis->Run()) {
    while (true) {

        std::cout<<"Before dostepdynamics \n";
        // std::cout<<"Avant dostepdynamics \n";
        sys.DoStepDynamics(timestep);
        // std::cout<<"Après dostepdynamics \n";

        
        // irr::video::IVideoDriver * vid_driver = vis->GetVideoDriver();
        
        #ifdef CHRONO_IRRLICHT
        std::cout<<"Avant begin scene \n";
        vis->BeginScene();
        std::cout<<"Après beginscene \n";
        vis->Render();
        // std::cout<<"Après render \n";
        vis->GetDevice()->run();
        // std::cout<<"Après getdevice \n";
        vis->EndScene();
        // std::cout<<"Apres endscene \n";
        #endif
        GetLog() << "\n" <<timestep;

        // realtime_timer.Spin(timestep);
    }

    return 0;
}
