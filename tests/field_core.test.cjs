const test=require('node:test');
const assert=require('node:assert/strict');
const F=require('../sentinel/assets/field_core.js');
const A=require('../sentinel/assets/navigation.js');

test('image-based terrain is deterministic and explicitly derived from pixels',()=>{
  const data=new Uint8ClampedArray(16*16*4);
  for(let y=0;y<16;y++)for(let x=0;x<16;x++){
    const i=(y*16+x)*4,v=x<8?0:255;data[i]=v;data[i+1]=v;data[i+2]=v;data[i+3]=255;
  }
  const heights=F.imageTerrain({data,width:16,height:16},16,4);
  assert.equal(heights.length,256);
  assert.ok(heights[15]>heights[0]+2);
  assert.deepEqual(heights,F.imageTerrain({data,width:16,height:16},16,4));
});

test('simulated scans reveal cells gradually without exposing full terrain early',()=>{
  const n=32,truth=Float64Array.from({length:n*n},(_,i)=>i%n>16?2:0);
  const events=F.simulateSurvey(truth,n,.5);
  assert.ok(events.length>9);
  assert.deepEqual(new Set(events.map(e=>e.drone)),new Set([0,1,2,3,4,5]));
  const count=e=>e.known.reduce((a,b)=>a+b,0);
  assert.ok(count(events[0])>0);
  assert.ok(count(events[0])<n*n/2);
  assert.ok(count(events.at(-1))>n*n*.8);
  assert.ok(events[0].height.some(Number.isNaN));
  const firstBefore=events[0].known.slice();
  assert.deepEqual(events[0].known,firstBefore);
});

test('five simulated observers contribute to one evidence replay',()=>{
  const n=64,events=F.simulateSurvey(new Float64Array(n*n),n,.5,5);
  assert.deepEqual(new Set(events.map(e=>e.drone)),new Set([0,1,2,3,4]));
  assert.ok(events.at(-1).known.reduce((sum,value)=>sum+value,0)>n*n*.8);
});

test('simulated evidence enters the same ground route planner',()=>{
  const n=32,truth=new Float64Array(n*n),last=F.simulateSurvey(truth,n,.5).at(-1);
  const grid=F.gridAt(last,n,.5);
  const costs=A.groundCostmap(grid,{radius:0,maxHeight:1,maxStep:.5,maxSlope:45});
  const result=A.astar(costs,[5,5,0],[25,25,0],grid);
  assert.ok(result.length>0);
  assert.ok(result.path.length>2);
});

test('semantic terrain uses assumed building height and road routing excludes non-road cells',()=>{
  const n=5,semantic=new Uint8Array(n*n).fill(1);semantic[12]=2;
  const height=F.semanticTerrain(semantic,n,4);
  assert.equal(height[0],0);assert.equal(height[12],4);
  const grid={n,resolution:1,height,known:new Uint8Array(n*n).fill(1),roughness:new Float64Array(n*n),semantic};
  const costs=A.groundCostmap(grid,{radius:0,roadOnly:true,maxHeight:10});
  assert.equal(costs.passable[12],0);
  assert.equal(costs.passable[11],1);
  const route=A.astar(costs,[0,2,0],[4,2,0],grid);
  assert.ok(route.length>4);
  assert.ok(!route.cellIds.includes(12));
});

test('image road corridors bridge a one-cell segmentation gap and snap endpoints',()=>{
  const n=9,semantic=new Uint8Array(n*n);
  for(let x=1;x<8;x++)if(x!==4)semantic[4*n+x]=1;
  semantic[3*n+4]=2;
  const grid={n,resolution:1,height:F.semanticTerrain(semantic,n,4),known:new Uint8Array(n*n),roughness:new Float64Array(n*n),semantic};
  const costs=A.imageRoadCostmap(grid,{roadBufferCells:1});
  const start=A.nearestPassable(costs,[0,4,0]),goal=A.nearestPassable(costs,[8,4,0]);
  const route=A.astar(costs,start.point,goal.point,grid);
  assert.ok(route.length>0);
  assert.ok(!route.cellIds.includes(3*n+4));
});

test('image routing can use costly open connectors while never crossing a building',()=>{
  const n=7,semantic=new Uint8Array(n*n);
  semantic[3*n+1]=1;semantic[3*n+5]=1;
  for(let y=2;y<=4;y++)semantic[y*n+3]=2;
  const grid={n,resolution:1,height:F.semanticTerrain(semantic,n,4),known:new Uint8Array(n*n).fill(1),roughness:new Float64Array(n*n),semantic};
  const costs=A.imageRoadCostmap(grid,{roadBufferCells:0});
  const route=A.astar(costs,[1,3,0],[5,3,0],grid);
  assert.ok(route.length>4);
  assert.ok(!route.cellIds.some(i=>semantic[i]===2));
  assert.ok(route.cost>route.length);
});

test('image terrain routing avoids forest and water while allowing costly low vegetation',()=>{
  const n=9,semantic=new Uint8Array(n*n);
  for(let x=0;x<n;x++)semantic[4*n+x]=1;
  // A forest wall leaves one low-vegetation crossing for the route.
  for(let y=0;y<n;y++)semantic[y*n+4]=3;
  semantic[5*n+4]=4;
  semantic[2*n+3]=5;
  const grid={n,resolution:1,height:F.semanticTerrain(semantic,n,4),known:new Uint8Array(n*n).fill(1),roughness:new Float64Array(n*n),semantic};
  const costs=A.imageRoadCostmap(grid,{roadBufferCells:0});
  const route=A.astar(costs,[0,4,0],[8,4,0],grid);
  assert.ok(route.cellIds.some(i=>semantic[i]===4));
  assert.ok(!route.cellIds.some(i=>semantic[i]===3||semantic[i]===5));
  assert.equal(costs.passable[4*n+4],0);
  assert.equal(costs.passable[2*n+3],0);
});

test('image endpoint snapping ignores tiny isolated prediction islands',()=>{
  const n=9,semantic=new Uint8Array(n*n);
  for(let x=0;x<n;x++)semantic[4*n+x]=1;
  semantic[8*n+8]=1;
  for(const [x,y] of [[7,7],[8,7],[7,8]])semantic[y*n+x]=2;
  const grid={n,resolution:1,height:F.semanticTerrain(semantic,n,4),known:new Uint8Array(n*n).fill(1),roughness:new Float64Array(n*n),semantic};
  const costs=A.imageRoadCostmap(grid,{roadBufferCells:0});
  const snapped=A.nearestPassable(costs,[8,8,0],12);
  assert.notDeepEqual(snapped.point,[8,8,0]);
  assert.equal(costs.component[snapped.point[1]*n+snapped.point[0]],costs.primaryComponent);
});
