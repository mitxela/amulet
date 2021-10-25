$fs=0.25;
$fa=3;

difference() {
  cylinder( 8, d=23 );
  union(){
    translate([0,0,-1]) cylinder( 12, d=19 );
    translate([0,0,1]) cylinder( 10, d=21 );
  }
}



translate([0,-15,4]) rotate([-90,0,0]){
  cylinder( 4, d=3 );
  translate([0,1.5,-1]) rotate([90,0,0]) difference() {
    cylinder( 3, d=5 );
    translate([0,0,-1]) cylinder( 5, d=2 );
  }
}