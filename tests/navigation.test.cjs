const test=require('node:test');
const assert=require('node:assert/strict');
const A=require('../sentinel/assets/navigation.js');

function flat(n){return{n,resolution:1,size:n,height:new Float64Array(n*n),known:new Uint8Array(n*n).fill(1),roughness:new Float64Array(n*n)}}

test('point import preserves holes and highest surface evidence',()=>{
 const p=A.parseXYZ('x,y,z\n0,0,0\n1,0,2\n1,0,3\n0,1,0');const g=A.gridFromPoints(p,1);
 assert.equal(g.known[3],0);assert.ok(Number.isNaN(g.height[3]));assert.equal(g.height[1],3);assert.equal(g.roughness[1],1);
});
test('ASCII PLY property order is respected and binary is rejected',()=>{
 const p=A.parseXYZ('ply\nformat ascii 1.0\nelement vertex 3\nproperty float z\nproperty float x\nproperty float y\nend_header\n3 1 2\n6 4 5\n9 7 8');assert.deepEqual(p[0],[1,2,3]);
 assert.throws(()=>A.parseXYZ('ply\nformat binary_little_endian 1.0\nend_header'),/ASCII/);
 assert.throws(()=>A.parseXYZ('x,y,z\n0,0,NaN\n1,0,0\n1,1,0'),/Invalid/);
});
test('A* finds optimal flat diagonal and cannot cut blocked corners',()=>{
 const g=flat(5),c=A.groundCostmap(g,{radius:0});const route=A.astar(c,[0,0,0],[4,4,0],g);
 assert.ok(Math.abs(route.cost-4*Math.SQRT2)<1e-8);
 g.known[1]=0;g.known[5]=0;assert.throws(()=>A.astar(A.groundCostmap(g,{radius:0}),[0,0,0],[4,4,0],g),/No connected/);
});
test('unknown wall and excessive terrain steps prohibit routes',()=>{
 const g=flat(5);for(let y=0;y<5;y++)g.known[y*5+2]=0;
 assert.throws(()=>A.astar(A.groundCostmap(g,{radius:0}),[0,2,0],[4,2,0],g),/No connected/);
 g.known.fill(1);for(let y=0;y<5;y++)g.height[y*5+2]=1;
 assert.throws(()=>A.astar(A.groundCostmap(g,{radius:0,maxStep:.3}),[0,2,0],[4,2,0],g),/No connected/);
});
test('robot radius closes a passage too narrow for the vehicle',()=>{
 const g=flat(7);g.known.fill(0);for(let y=0;y<7;y++)g.known[y*7+3]=1;
 assert.equal(A.groundCostmap(g,{radius:.2}).passable[3*7+3],1);
 assert.equal(A.groundCostmap(g,{radius:.6}).passable[3*7+3],0);
});
test('3D routing uses observed free air and can change altitude',()=>{
 const g=flat(3);g.voxel={shape:[3,3,3],labels:new Array(27).fill(-1)};
 const id=(x,y,z)=>(z*3+y)*3+x;for(const p of [[0,1,1],[0,1,2],[1,1,2],[2,1,2],[2,1,1]])g.voxel.labels[id(...p)]=0;
 const r=A.astar(A.airCostmap(g,{radius:0}),[0,1,1],[2,1,1]);assert.equal(r.path.length,5);assert.equal(r.length,4);
 g.voxel.labels[id(1,1,2)]=1;assert.throws(()=>A.astar(A.airCostmap(g,{radius:0}),[0,1,1],[2,1,1]),/No connected/);
 assert.throws(()=>A.airCostmap(flat(3)),/endpoint-only/);
});
test('coordinate anchor and WGS84 conversion preserve origin and direction',()=>{
 const o=A.parseOrigin('https://www.google.com/maps/@37.33,-121.89,18z');assert.deepEqual(o,{lat:37.33,lon:-121.89});
 const p=A.localToLonLat(0,0,o);assert.ok(Math.abs(p[0]-o.lon)<1e-8);assert.ok(Math.abs(p[1]-o.lat)<1e-8);
 assert.ok(A.localToLonLat(100,0,o)[0]>o.lon);assert.ok(A.localToLonLat(0,100,o)[1]>o.lat);
 assert.throws(()=>A.parseOrigin('90,0'),/latitude/);
});
