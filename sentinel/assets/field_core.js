/* Browser-side illustrative image terrain and simulated overhead survey.
   A single RGB image is NOT a measured elevation model. */
(function(root,factory){if(typeof module==='object'&&module.exports)module.exports=factory();else root.FieldCore=factory();})(typeof globalThis!=='undefined'?globalThis:this,function(){
'use strict';
function imageTerrain(image,n=64,amplitude=4){
  if(!image||!image.data||image.width<2||image.height<2||n<8||n>128||!Number.isFinite(amplitude)||amplitude<0||amplitude>30)throw Error('Invalid image or terrain settings.');
  const raw=new Float64Array(n*n),src=image.data,w=image.width,h=image.height;
  for(let y=0;y<n;y++)for(let x=0;x<n;x++){
    const px=Math.min(w-1,Math.floor((x+.5)/n*w)),py=Math.min(h-1,Math.floor((1-(y+.5)/n)*h)),i=(py*w+px)*4;
    raw[y*n+x]=(.2126*src[i]+.7152*src[i+1]+.0722*src[i+2])/255;
  }
  // Blur pixel/compression noise; this remains an illustrative height hypothesis.
  const smooth=new Float64Array(n*n);
  for(let y=0;y<n;y++)for(let x=0;x<n;x++){
    let sum=0,weight=0;
    for(let dy=-2;dy<=2;dy++)for(let dx=-2;dx<=2;dx++){
      const xx=Math.min(n-1,Math.max(0,x+dx)),yy=Math.min(n-1,Math.max(0,y+dy)),a=dx===0&&dy===0?4:1;
      sum+=raw[yy*n+xx]*a;weight+=a;
    }
    smooth[y*n+x]=amplitude*sum/weight;
  }
  return smooth;
}
function simulateSurvey(truth,n,resolution=.5,droneCount=6){
  if(truth.length!==n*n||!Number.isFinite(resolution)||resolution<=0)throw Error('Terrain and grid dimensions disagree.');
  if(!Number.isInteger(droneCount)||droneCount<1||droneCount>8)throw Error('Use between one and eight simulated observers.');
  // Contiguous swaths tile the entire image, including its perimeter. These
  // synthetic samples visualize existing image predictions, not new measurements.
  const stride=Math.max(1,Math.ceil(n/32)),lanes=Array.from({length:droneCount},(_,id)=>(id+.5)*n/droneCount-.5),rows=[];
  for(let y=0;y<n;y+=stride)rows.push(y);
  const height=new Float64Array(n*n).fill(NaN),known=new Uint8Array(n*n),count=new Uint8Array(n*n),events=[];
  for(let turn=0;turn<rows.length;turn++)for(let drone=0;drone<droneCount;drone++){
    const site=[lanes[drone],Math.min(n-1,rows[turn]+stride-1)];let returns=0,newCells=0;
    for(let y=rows[turn];y<Math.min(n,rows[turn]+stride);y++)for(let x=Math.floor(drone*n/droneCount);x<Math.floor((drone+1)*n/droneCount);x++){
      const i=y*n+x;if(!Number.isFinite(truth[i]))continue;
      const noisy=truth[i]+.035*Math.sin(i*1.31+turn*2.07+drone);
      height[i]=count[i]?(height[i]*count[i]+noisy)/(count[i]+1):noisy;
      if(!count[i]){known[i]=1;newCells++;}count[i]=Math.min(255,count[i]+1);returns++;
    }
    events.push({drone,x:site[0],y:site[1],returns,newCells,height:Float64Array.from(height),known:Uint8Array.from(known)});
  }
  return events;
}
function gridAt(event,n,resolution){
  return {n,resolution,size:n*resolution,height:event.height,known:event.known,
    roughness:new Float64Array(n*n),offset:[0,0],source:'image-based simulated survey',voxel:null};
}
function semanticTerrain(labels,n,buildingHeight=4){
  if(!labels||labels.length!==n*n||!Number.isFinite(buildingHeight)||buildingHeight<0||buildingHeight>30)
    throw Error('Invalid semantic grid or assumed building height.');
  // These are display-only height assumptions. The route costmap governs
  // traversability; a single overhead image does not measure canopy or slope.
  return Float64Array.from(labels,v=>v===2?buildingHeight:v===3?2:v===4?.45:v===1?0:.12);
}
function applySemanticCrop(labels,n,bounds,cropLabels,cropN=128){
  if(!labels||labels.length!==n*n||!cropLabels||cropLabels.length!==cropN*cropN||!Number.isInteger(cropN)||cropN<1)
    throw Error('Invalid semantic crop.');
  const{minX,maxX,minY,maxY}=bounds||{};
  if(![minX,maxX,minY,maxY].every(Number.isInteger)||minX<0||minY<0||maxX>=n||maxY>=n||minX>maxX||minY>maxY)
    throw Error('Invalid semantic crop bounds.');
  const changed=[],before=new Uint32Array(6),after=new Uint32Array(6),width=maxX-minX+1,height=maxY-minY+1;
  for(let y=minY;y<=maxY;y++)for(let x=minX;x<=maxX;x++){
    const u=Math.min(cropN-1,Math.floor((x-minX)/width*cropN)),v=Math.min(cropN-1,Math.floor((y-minY)/height*cropN)),i=y*n+x,next=Number(cropLabels[v*cropN+u]);
    if(labels[i]>=0&&labels[i]<before.length)before[labels[i]]++;
    if(next>=0&&next<after.length)after[next]++;
    if(labels[i]!==next){labels[i]=next;changed.push(i);}
  }
  return{changed,before:Array.from(before),after:Array.from(after)};
}
return{imageTerrain,simulateSurvey,gridAt,semanticTerrain,applySemanticCrop};
});
