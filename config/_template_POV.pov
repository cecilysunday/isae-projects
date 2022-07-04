//
// Script to be executed with POV-Ray for rendering a frame of a Chrono::Engine generated animation.
// Most often, in POV-Ray, you run the .INI file, that calls this file multiple times to render
// the many frames of a complete animation.
//
// Plase do not modify the _template_POV.pov , unless you know exactly what you are
// doing, because _template_POV.pov is used by default to build POV scenes by the postprocessing
// unit of Chrono::Engine


#include "debug.inc"
#include "colors.inc" 
#include "textures.inc"           
           
// 
// Function to generate a rotation matrix from a C::E quaternion data   
// 


#macro quatRotation(q)
    #local aa = q.x*q.x;
    #local ab = q.x*q.y;
    #local ac = q.x*q.z;
    #local ad = q.x*q.t;
    #local bb = q.y*q.y;
    #local bc = q.y*q.z;
    #local bd = q.y*q.t;
    #local cc = q.z*q.z;
    #local cd = q.z*q.t;
    #local dd = q.t*q.t;
    #local qq = aa+bb+cc+dd;
    matrix <(aa+bb-cc-dd)/qq, 2*(bc+ad)/qq, 2*(bd-ac)/qq,
                2*(bc-ad)/qq, (aa-bb+cc-dd)/qq, 2*(cd+ab)/qq,
                2*(bd+ac)/qq, 2*(cd-ab)/qq, (aa-bb-cc+dd)/qq,
                0, 0, 0>
#end

#declare apx=0;
#declare apy=0;
#declare apz=0;
#declare aq0=0;
#declare aq1=0;
#declare aq2=0;
#declare aq3=0;  


// Defaults   
#default {
  pigment {rgb <1,1,1>}
}
      
        
// A macro to create grids
#macro Raster(RScale, RLine, lcolor)
pigment{
   gradient x scale RScale
   color_map{
     [0.000   color lcolor]
     [0+RLine color lcolor]
     [0+RLine color rgbt<1,1,1,1>]
     [1-RLine color rgbt<1,1,1,1>]
     [1-RLine color lcolor]
     [1.000   color lcolor]
            }
       } 
#end

#macro Grid(Scale,
            Line_thickness,
            Line_color,
            Plane_color)
plane{<0,1,0>, 0
      texture{ pigment{color Plane_color } } 
      texture{ Raster(Scale,  Line_thickness*0.5, Line_color) } 
      texture{ Raster(Scale,  Line_thickness*0.5, Line_color) rotate<0,90,0> }
      no_shadow 
     } 
#end
     

       
// -----------------------------------------------------------------------------
// OBJECTS TO BE RENDERED ARE AUTOMATICALLY INSERTED AFTER THIS LINE
// THANKS TO THE POSTPROCESSING UNIT OF CHRONO::ENGINE. YOU SHOULD NOT NEED TO
// MODIFY WHAT FOLLOWS.

