// Create a Clifford Algebra with 2,0,1 metric.
Algebra(2,0,1,()=>{
  let point = (x, y) => 1e12 - x*1e02 + y*1e01;
  
  let box = [
    point( 0.5, 0.5),
    point(-0.5, 0.5),
    point(-0.5,-0.5),
    point( 0.5,-0.5)
  ];
  
  let M = 1 + 0.5*1e02;
  let B = 1e02 + 1e12;
  let dt = 0.01;

  document.body.appendChild(this.graph(()=>{
    M = M - 0.5*M*B*dt
    B = B + 0.5 * !(B*!B - !B*B)*dt

    return [
      0x1111aa, M >>> box
    ];   
  },{animate:true, lineWidth:3, labels:true, grid:true}));
});
