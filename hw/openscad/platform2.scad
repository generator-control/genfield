/* File: platform2.scad
 * Platform to hold f103Ard board with fethe subboard, mounted above 
 * Author: deh
 * Latest edit: 20200112
 */
 $fn = 30;
 
 thick = 4;
 bwid  = 63;
 
 bleng = 140.0;
 bpbase = [bleng, bwid, thick]; // BP Motherboard base
 fleng  = 46.5;
 fbase  = [bwid, fleng, thick]; // fethe subboard base
 
 // BP Motherboard (f103Ard) hole locations
 delx = (1.006);   // X calibration adjustment
 dely = (1.000);   // Y calibration adjustment
 holeb1 = [  3.8*delx , 42.0,  0];  
 holeb2 = [ 99.9*delx,   6.5,  0];
 holeb3 = [ 99.5*delx,  58.0,  0];
 holeb4 = [135.6*delx,   3.0,  0];
 holeb5 = [134.5*delx,  33.6,  0];
 
 // FETHE pcb hole locations
 holef1 = [ 3.6, 42.9,  0];
 holef2 = [51.4,  2.5,  0];
 holef3 = [57.2,  6.8,  0];
 holef4 = [57.2, 29.6,  0];
 holef5 = [57.2, 43.0,  0];
 
 // FETHE board positionings
 transf1 = [138.0-46.3,bwid, 20.25];
 
 /* ***** rounded_rectangle_hull ********************
 * wid  = width, x direction 
 * slen = length, y direction
 * ht   = height (or thickness if you prefer)
 * rad  = z axis, radius of corner rounding
*/
module rounded_rectangle_hull(wid,slen,ht,rad)
{
 hull()
 {    
    translate([-wid/2+rad,rad,0])
        cylinder(r=rad,h=ht,center=false);

   translate([ wid/2-rad,rad,0])
        cylinder(r=rad,h=ht,center=false);

    translate([-wid/2+rad,slen-rad,0])
        cylinder(r=rad,h=ht,center=false);
    
    translate([ wid/2-rad,slen-rad,0])
        cylinder(r=rad,h=ht,center=false);
 }
}
  
 // Holes to mount top plate
 module holetop(a)
 {
     translate(a)
        cylinder (d=3.5,h=50,center=true);
 }
 
 //
 module post(a)
 {
     translate(a)
     {
         difference()
        {
            cylinder(d1=8.0, d2=5.5,h=(4+thick),center=false);      
            translate([0,0,(4+thick)-3.5])
            cylinder(d=1,h=10,center=false);
        }
     }   
 }
 module fposth(a,dd)
 {
     translate(a)
        cylinder(d=dd,h=10,center=false);
 }
 spy = 5;
 spz = 20;
 spoy = 6;
 spyy = spy+spoy;
 
 // Extension of fethe plate
 module ftab(a)
 {
     translate(a)
     difference()
     {
          translate([-spyy,0,0])
                cube([spyy,fleng,thick],center=false);
         union()
         {
           holetop([-8.5,      7,0]);
           holetop([-8.5,fleng-7,0]);
         }
     }
     
 }
 
 // fethe subboard platform
 module fplat(a)
 {
   translate(a)
   {
     rotate([0,0,-90])
     {
        difference()
        {
            union()
            {
                cube(fbase,false);
                post(holef1);
                post(holef2);
                post(holef3);
                post(holef4);
                post(holef5);
                
                ftab([0,0,0]);
                rotate([0,0,180])
                ftab([-bwid,-fleng,0]);
            }
            union()
            {
                translate([28,23,25])
                cube([35,35,50],true);
                
                fposth(holef1,2.8);
                fposth(holef2,2.8);
                fposth(holef3,2.8);
                fposth(holef4,2.8);
                fposth(holef5,2.8);         
            }
        }
    }
  }
 }
 
 module pillarhole(a)
 {
     translate(a)
     {
        translate([0,0,0])
        cylinder(d=2.8,h = 50,center = false);
     }
 }
 
 // Side supports for fethe on top
 module sidepillars(a,aa,x)
 {
     b = [x,spy,spz];
     translate(a)
     rotate(aa)
     translate([-x/2,0,0])
     {
         difference()
         {
             union()
             {
                 translate([0,spoy,0])
                   cube(b,center=false);
              
                 cube([x,spoy,thick],center=false);
                 
             }
             union()
             {
                 pillarhole([      7,spoy+spy/2,0]);
                 pillarhole([fleng-7,spoy+spy/2,0]);
             }
         }
     }  
 }
 module rounded_bar(d, l, h)
{
    // Rounded end
    cylinder(d = d, h = h, center = false);
    // Bar
    translate([0, -d/2, 0])
       cube([l, d, h],false);
}
// Mounting tab on base plate
 module btab(a)
 {
   translate(a)
   difference()
   {   
      rounded_bar(12,12,thick);
      cylinder(d = 3.2, h = 10, center = false);
   }
     
 }
 module base()
 {
     difference()
     {
        union()
        {
            // Base plate with f103Ard mounting posts
            cube(bpbase,false);
            
            translate([7.5,-10,0])
              cube([87,spyy,thick],center=false);
            
            translate([7.5,bwid,0])
              cube([87,spyy,thick],center=false);
            
            // Posts 
            post(holeb1);
            post(holeb2);
            post(holeb3);
            post(holeb4);
            post(holeb5);

            // Side pillars for fethe mount on top
        sl = fleng;
            sidepillars([bleng-sl/2,bwid,0],[0,0,  0],sl);
            sidepillars([bleng-sl/2,   0,0],[0,0,180],sl);
            
            // Mounting tabs on base plate
            translate([6,-4,0])
             rotate([0,0,90])
              btab([0,0,0]);
            translate([6,bwid+4,0])
             rotate([0,0,-90])
              btab([0,0,0]);
            translate([bleng+3,bwid/2,0])
             rotate([0,0,180])
              btab([0,0,0]);

        }
        union()
        {
            // Cutouts        
            translate([87,12,-1])
              rounded_rectangle_hull(82,40,50,12);
            translate([52,-2,-1])
              rounded_rectangle_hull(83,67,50,12);
            
            // Holes for pcb mounting screws
            fposth(holeb1,2.9);
            fposth(holeb2,2.9);
            fposth(holeb3,2.9);
            fposth(holeb4,2.9);
            fposth(holeb5,2.9);
        }
     }
 }
 
 module total()
 {
     difference()
     {
         union()
         {
             base();
 //           fplat(transf1);
         }
         union()
         {
         }
     }
 }
total();

 
// fplat([-55,64,0]);

 
