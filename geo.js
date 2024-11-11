
var Geo=(function() {

    var tol=0.0000000000001

    
	function round(x) {
		return Math.round(x/tol)*tol
	}
	
	function radians (x) {
		return x/180*Math.PI
	}
	
    function degrees (x) {
		return x*180/Math.PI
	}
	
	function rotation3d(u,p,a,pu=[0,0,0]) {
        var cos=Math.cos(a)
        var sin=Math.sin(a)
        var omc=1-cos
        var p_=[p[0]-pu[0],p[1]-pu[1],p[2]-pu[2]]
        
        var x=(u[0]**2  *omc     +cos)*p_[0] +(u[0]*u[1]*omc-u[2]*sin)*p_[1] +(u[0]*u[2]*omc+u[1]*sin)*p_[2] +pu[0]
        var y=(u[1]*u[0]*omc+u[2]*sin)*p_[0] +(u[1]**2  *omc     +cos)*p_[1] +(u[1]*u[2]*omc-u[0]*sin)*p_[2] +pu[1]
        var z=(u[2]*u[0]*omc-u[1]*sin)*p_[0] +(u[2]*u[1]*omc+u[0]*sin)*p_[1] +(u[2]**2  *omc     +cos)*p_[2] +pu[2]

        return [x,y,z]
	}

    function angleDiff3d(vo_,vr_,toUnit=false) {
		var vo,vr	
	    if (toUnit==true) {
            vo=UV(vo_)
            vr=UV(vr_)
        } else {
            vo=[vo_[0],vo_[1],vo_[2]]
            vr=[vr_[0],vr_[1],vr_[2]]
		}
        var r=VP(vo,vr,true,true)
        var u=r[0]
        var a=Math.asin(Math.max(-1,Math.min(r[1],1)))
        var p=rotation3d(u,vo,a)
        if (interval(vr,p)>tol)
            a=Math.PI-a
        return [a,u]
	}

    function angle(x,z,AllowNegative=false) {
        var a
		if (x==0)
            if (z>0)
                a=Math.PI*0.5
            else
                a=Math.PI*1.5
        else {
            a=Math.atan(z/x)
            if (x<0)
                a+=Math.PI
		}
        if (AllowNegative==false && a<0)
            a+=Math.PI*2
        return a
	}

    function interval(a,b=[0,0,0]) {
        var s=0
        var l=a.length
        for (var i=0;i<l;i++)
            s+=Math.pow(a[i]-b[i],2)
        return Math.pow(s,0.5)
	}

    function translate(V,d,P=[0,0,0]) {
        var l=V.length
        var r=[]
        for (var i=0;i<l;i++)
            r.push(P[i]+V[i]*d)
        return r
	}

    function UV(r,withIntrval=false) {
        var l=r.length
        var scalar=interval(r)
        if (scalar==0)
            return [0,0,0]
        var R=[]
        for (var i=0;i<l;i++)
            R.push(r[i]/scalar)
        if (withIntrval==true)
            return [R,scalar]
        else
            return R      
	}
	
	function VP(Xvector,Zvector,toUnit=false,withScalar=false) {
        var r=[Xvector[1]*Zvector[2]-Xvector[2]*Zvector[1],Xvector[2]*Zvector[0]-Xvector[0]*Zvector[2],Xvector[0]*Zvector[1]-Xvector[1]*Zvector[0]]
 		if (toUnit==true) {
            var scalar=interval(r)
			if (scalar>0)
				r=[r[0]/scalar,r[1]/scalar,r[2]/scalar]
			else
				r=[0,0,0]
			if (withScalar==false)
                return r
            else
                return [r,scalar]
        } else
            return r
	}
	
	function rv(p) {
		var lat=radians(p[0])
        var lng=radians(p[1])
        var cos=Math.cos(lat)
        return [Math.cos(lng)*cos,Math.sin(lng)*cos,Math.sin(lat)]
	}
	
	function geo(r) {
        var lat=round(degrees(Math.asin(r[2]))),lng
        if (Math.abs(lat)==90)
            lng=0
        else {
            var r_=UV([r[0],r[1],0])
            lng=round(degrees(angle(r_[0],r_[1])))
			if (lng==360)
				lng=0
		}
		return [lat,lng]
	}
	
	function axis(r,a) {
		var par
		if (interval([r[0],r[1]])>tol) {
            par=VP(r,[0,0,1],true)
            a=-a
        } else
			if (r[2]>0) {
				par=[0,1,0]
	            a=a
			} else {
				par=[0,-1,0]
	            a=-a
			}
        return rotation3d(r,par,radians(a))
	}
	
	function diff(r,v,ax) {
		var d=angleDiff3d(r,v)
		if (interval(d[1],ax)>1)
			d[0]=Math.PI*2-d[0]
		if (Math.abs(d[0]-Math.PI*2)<tol)
			d[0]=0
		return d[0]
	}
	
	return {
		sphericalTwoGreateCirclesClosestIntersectionPointCoordinates: function(p1,a1,p2,a2,withDistance=false) {    
			
			var r1=rv(p1)
			var ax1=axis(r1,a1)

			var r2=rv(p2)
			var ax2=axis(r2,a2)
			
			var v1=VP(ax1,ax2)
			if (interval(v1)<tol)
				return false
			v1=UV(v1)
			
			var v2=translate(v1,-1)
			
			var d11=diff(r1,v1,ax1)
			var d12=diff(r1,v2,ax1)
			var d21=diff(r2,v1,ax2)
			var d22=diff(r2,v2,ax2)
			
			if (d11+d21-d12-d22>tol || (Math.abs(d11+d21-d12-d22)<tol && d11-d12>tol)) {
				v1=v2
				d11=d12
				d21=d22
			}

			var result=geo(v1)
			
			if (withDistance==false)
				return result
			else
				return [result,[round(degrees(d11)),geo(ax1)],[round(degrees(d21)),geo(ax2)]]
		}
	}
})()
