// Create a Clifford Algebra with 2,0,1 metric.
Algebra(2,0,1,()=>{
  let point = (x, y) => 1e12 - x*1e02 + y*1e01;
  let dist_pl = (p, l) => (p^l).e012;
  let wrapped_pairs = a => a.map((x, i)=>[x, a[(i+1)%a.length]]);
  let check_wall_collision = (obj, walls) => {
    // If there is a collision return its normal line. Otherwise return null.
    let world_points = obj.M >>> obj.points;
    for (let p=0; p<world_points.length; p++) {
      let P = world_points[p];
      for (let w=0; w<walls.length; w++) {
        let wall = walls[w];
        let d = dist_pl(P, wall);
        if (d < 0) {
          let n = (wall | 1e12) / 1e12;
          return n | P;
        }
      }
    }

  }
  
  let box_points = [
    point( 0.5, 0.5),
    point(-0.5, 0.5),
    point(-0.5,-0.5),
    point( 0.5,-0.5)
  ];
  let objects = [{
    points: box_points,
    M: 1 + 0.5*1e02,
    B: 1e02 + 1e12,
    color: 0x1111aa
  },
  {
    points: box_points,
    M: 1 + 0.75*1e01,
    B: 1e01 - 1e12,
    color: 0x11aa11
  }];

  // let wall_points = box_points.reverse().map(p=>1e12 + 5*p.e02*1e02 + 5*p.e01*1e01);
  let wd = 2.5;
  let walls = [
    -1e1 + wd*1e0,
     1e1 + wd*1e0,
    -1e2 + wd*1e0,
     1e2 + wd*1e0,
  ]


  let dt = 0.01;

  document.body.appendChild(this.graph(()=>{
    for(let o=0; o<objects.length; o++) {
      let M = objects[o].M, B = objects[o].B;
      M = M - 0.5*M*B*dt
      B = B + 0.5 * !(B*!B - !B*B)*dt
      objects[o].M = M;
      objects[o].B = B;
    }

    return [
      // Objects
      ...objects.map(o=>[o.color, o.M>>>o.points]).flat(),

      // Walls
      0xaa1111, ...walls,
    ];
  },{animate:true, lineWidth:3, labels:true, grid:true}));
});
