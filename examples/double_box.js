Algebra(2,0,1,()=>{
  let point = (x, y) => 1e12 - x*1e02 + y*1e01;
  let dist_pl = (p, l) => (p.Normalized^l.Normalized).e012;
  let comm = (x, y) => 0.5*(x*y - y*x);
  let total_E = objs => objs.map(o => (o.I(o.B)^o.B).e012).reduce((accum, x) => accum + x);
  let wrapped_pairs = a => a.map((x, i) => [x, a[(i+1) % a.length]]);

  let check_wall_collision = (obj, walls) => {
    // If there is a collision, correct it and return a [point, normal] pair. Otherwise return null.
    let world_points = obj.M >>> obj.points;
    let worst_P=null, worst_wall=null, worst_d=0;
    for (let p=0; p<world_points.length; p++) {
      let P = world_points[p];
      for (let w=0; w<walls.length; w++) {
        let wall = walls[w];
        let d = dist_pl(P, wall);
        if (d < worst_d) {
          worst_P = P; worst_wall = wall; worst_d = d;
        }
      }
    }

    if (worst_d < 0) {
      let n = -(worst_wall | 1e12) / 1e12;
      obj.M = (1-0.5*worst_d*1e0*n)*obj.M;
      return [worst_P, n];
    }
  }

  let check_inter_object_collision = (obj1, obj2) => {
    // If there is a collision, correct it and return a [point, edge] pair. Otherwise return null. 
    // NOTE: Return type broken for troubleshooting
    let points1 = obj1.points.map(p=>obj1.M >>> p);
    let edges1 = wrapped_pairs(points1).map(([a,b]) => a&b);

    let points2 = obj2.points.map(p=>obj2.M >>> p);
    let edges2 = wrapped_pairs(points2).map(([a,b]) => a&b);

    let firstMax = a => a.reduce((max, current) => max[0] < current[0] ? current : max);
    let firstMin = a => a.reduce((min, current) => min[0] > current[0] ? current : min);

    let minPenPointAndEdge = (myEdges, otherPoints) => {
      let minPen = Infinity, minPenPoint = null, minPenEdge = null;
      for (let e=0; e<myEdges.length; e++) {
        let edge = myEdges[e];
        let edgeMinDist = Infinity;
        let edgeMinPenPoint = null;
        for (let p=0; p<otherPoints.length; p++) {
          let P = otherPoints[p];
          let d = dist_pl(P, edge);
          if (d < edgeMinDist) {
            edgeMinDist = d;
            edgeMinPenPoint = P;
          }
        }
        if (edgeMinDist > 0) {
          // This is a separating edge
          return null;
        } else if (Math.abs(edgeMinDist) < minPen) {
          // This edge is not separating but has a lower "penetration level" than the previous lowest
          minPen = Math.abs(edgeMinDist);
          minPenPoint = edgeMinPenPoint;
          minPenEdge = edge;
        }
      }
      return [minPen, minPenPoint, minPenEdge];
    }

    let obj1MinPenPointAndEdge = minPenPointAndEdge(edges1, points2);
    let obj2MinPenPointAndEdge = minPenPointAndEdge(edges2, points1);

    if (obj1MinPenPointAndEdge === null || obj2MinPenPointAndEdge == null) {
      return null;
    }

    let [minPen1, minPenPoint1, minPenEdge1] = obj1MinPenPointAndEdge;
    let [minPen2, minPenPoint2, minPenEdge2] = obj2MinPenPointAndEdge;
    if (minPen1 < minPen2) {
      let n = (minPenEdge1 | 1e12) / 1e12;
      obj1.M = (1 + 1e0*n*minPen1/4) * obj1.M;
      obj2.M = (1 - 1e0*n*minPen1/4) * obj2.M;
      return [minPenPoint1, n]
    }
    let n = (minPenEdge2 | 1e12) / 1e12;
    obj1.M = (1 - 1e0 * n*minPen2/4) * obj1.M;
    obj2.M = (1 + 1e0 * n*minPen2/4) * obj2.M;
    return [minPenPoint2, -n]
  }

  let rebound = (P, n, obj1, obj2, r) => {
    // `n` should be oriented in the direction which obj1 should rebound.
    // This function will work in the world frame, so we need to convert `obj.B` and `obj.Iinv` to world frame
    console.log('Rebounding with restitution:', r);
    let M1 = obj1.M;
    let B1 = M1 >>> obj1.B;
    let I1inv = X=> M1 >>> obj1.Iinv(~M1 >>> X);

    let I2inv, M2, B2;
    if (obj2) {
      M2 = obj2.M;
      B2 = M2 >>> obj2.B;
      I2inv = X=> M2 >>> obj2.Iinv(~M2 >>> X);
    } else {
      M2 = 0;
      B2 = 0;
      I2inv = (_) => 0;
    }

    let N = n | P;
    let j = -(1+r)*((P & comm(P, B1-B2)) | ~N) / ((P & comm(P, I1inv(N)-I2inv(N))) | ~N); // From https://bivector.net/PGAdyn.pdf page 46

    // Convert results back to body frame before storing to object
    obj1.B = ~M1 >>> (B1 + j*I1inv(N));
    if (obj2) {
      obj2.B = ~M2 >>> (B2 - j*I2inv(N));
    }
  }
  
  let box_points = [
    point( 0.5, 0.5),
    point(-0.5, 0.5),
    point(-0.5,-0.5),
    point( 0.5,-0.5)
  ];

  let objects = [
    {
      points: box_points,
      I: x => x.Dual,
      Iinv: x => x.UnDual,
      M: 1 + 0.25*1e02 - 0.25*1e01,
      B: 1e02 + 1e12,
      color: 0x1111aa
    },
    {
      points: box_points,
      I: x => x.Dual,
      Iinv: x => x.UnDual,
      M: 1 + 0.5*1e01,
      B: 1e01 - 1e12,
      color: 0x11aa11
    }
  ];

  let initial_E = total_E(objects);

  let wd = 1.8;
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
      M = M - 0.5*M*B*dt;
      B = B + 0.5 * !(B*!B - !B*B)*dt;
      objects[o].M = M;
      objects[o].B = B;
    }

    for(let o=0; o<objects.length; o++) {
      let obj = objects[o];
      let col_params = check_wall_collision(obj, walls);
      if (col_params) {
        rebound(...col_params, obj, null, initial_E / total_E(objects));
      } 
    }

    let collisionEdges = [];
    for(let o1=0; o1<objects.length-1; o1++) {
      for(let o2=o1+1; o2<objects.length; o2++) {
        let col = check_inter_object_collision(objects[o1], objects[o2]);
        if (col) {
          // let [P, n] = col;
          // rebound(P, -n, objects[o1], objects[o2], 0.9);
        }
      }
    }

    return [
      // Objects
      ...objects.map(o=>[o.color, o.M>>>o.points]).flat(),

      // Walls
      0xaa1111, ...walls,
    ];
  },{animate:true, lineWidth:3, labels:true, grid:true}));
});
