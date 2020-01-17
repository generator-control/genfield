/* File: pwr24v.scad
 * Platform to hold 24v xfrmr and dc-dc 
 * Author: deh
 * Latest edit: 20200113
 */
 $fn = 30;
 
 thick = 4;

// Base
bx = 110; // Length
by = 85;  // Width
bt = 4;   // Thickness

// Transformer
tfhy = 58;   // Mtg hole spacing
tfhd = 3.5;  // Mtg hole diam

bhox = 3; // Base mtg hole offset from edge: x
bhoy = 3; // Base mtg hole offset from edge: y
module bhole(a)
{
	translate(a)
		cylinder(d =3.6,h=30,center=false);
}

acpx = 4;
acpy = 6;
acpz = 30;

module acpost(a)
{
	translate(a)
	{
		difference()
		{
			union()
			{
			}
			union()
			{
			}
		}
	}
}
dcpx = 4;
dcpy = 6;
dcpz = 45;

module dcpost(a)
{
	translate(a)
	{
		difference()
		{
			union()
			{
			}
			union()
			{
			}
		}
	}
}

module base()
{  
	difference()
	{
		union()
		{
			cube([bx,by,bt],center=false);
		}
		union()
		{
			// Base plate mounting holes
			bhole([   bhox,   bhoy,0]);
			bhole([bx-bhox,by-bhoy,o]);
			bhole([   bhox,by-bhoy,o]);
			bhole([bx-bhox,   bhoy,o]);

			// AC cord pcb posts


			// Rectified & dc-dc swrs post

	}
}

base();


