export const GAITS=Object.freeze({stand:[0,0,0],forward_slow:[.075,0,0],forward:[.12,0,0],backward:[-.15,0,0],left:[0,.07,0],right:[0,-.055,0],turn_left:[0,0,.7],turn_right:[0,0,-.7]});
export const LABELS={stand:'静止',forward_slow:'低速前進',forward:'前進',backward:'後退',left:'左移動',right:'右移動',turn_left:'左旋回',turn_right:'右旋回'};
export const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export function snapJoystick(x,y,slow=true){if(!Number.isFinite(x)||!Number.isFinite(y)||Math.hypot(x,y)<.23)return 'stand';return Math.abs(y)>=Math.abs(x)?(y<0?(slow?'forward_slow':'forward'):'backward'):(x<0?'left':'right');}
export class CommandRamp{
 constructor(){this.reset();}
 reset(){this.value=[0,0,0];this.from=[0,0,0];this.target=[0,0,0];this.mode='stand';this.elapsed=.5;}
 set(mode){if(!Object.hasOwn(GAITS,mode))throw new Error('Unknown gait command');if(mode===this.mode)return;this.mode=mode;this.from=this.value.slice();this.target=GAITS[mode].slice();this.elapsed=0;}
 step(dt=.02){this.elapsed=Math.min(.5,this.elapsed+dt);const f=this.elapsed/.5;this.value=this.target.map((x,i)=>this.from[i]+(x-this.from[i])*f);return this.value;}
}
