



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
	
    function VD(V1,V2) {
        var l=V1.length
        var r=[]
        for (var i=0;i<l;i++)
            r.push(V1[i]-V2[i])
        return r
	}
	
	function rv(p) {
		var lat=radians(p[0])
        var lng=radians(p[1])
        var cos=Math.cos(lat)
        return [Math.cos(lng)*cos,Math.sin(lng)*cos,Math.sin(lat)]
	}
	
	function geo(r) {
        var lat=degrees(Math.asin(r[2])),lng
        if (Math.abs(lat)==90)
            lng=0
        else {
            var r_=UV([r[0],r[1],0])
            lng=degrees(angle(r_[0],r_[1]))
			if (lng==360)
				lng=0
		}
		return [lat,lng]
	}
	
	function distanceToCrossPoint(R,v) {
		var d=angleDiff3d(R.point.radius,v)
		if (interval(d[1],R.axis.radius)>1)
			d[0]=Math.PI*2-d[0]
		if (Math.abs(d[0]-Math.PI*2)<tol)
			d[0]=0
		return d[0]
	}
	
	function checkPoint(p) {
		var P
		if (p instanceof Array)
			P=new Point(p)
		else
			P=p[0]
		return P
	}
	function checkRay(p) {
		var R
		if (p instanceof Array)
			R=new Ray(p[0],p[1])
		else
			R=p
		return R
	}
	
	var Point = function(location,radius) {
		if (location!=false) {
			this.radius=rv(location)
			this.location=location
		} else if (radius!=false) {
			this.radius=radius
			this.location=geo(radius)			
		}
		this.locationRounded=[round(this.location[0]),round(this.location[1])]
	}
	
	var Ray = function(p,a) {
		this.point=checkPoint(p)
		this.azimuth=a
		this.axis = new Point(false,Ray.axis.call(this))
	}
	Ray.axis = function() {
		var par,a,k=interval([this.point.radius[0],this.point.radius[1]])
		if (k>tol) {
            par=VP(this.point.radius,[0,0,1],true)
			a=-this.azimuth
        } else
			if (this.point.radius[2]>0) {
				par=[0,1,0]
	            a=this.azimuth
			} else {
				par=[0,-1,0]
	            a=-this.azimuth
			}
        return rotation3d(this.point.radius,par,radians(a))
	}
	Ray.prototype.travelRadias = function (d,p=false) { //travel angle in radians, from point as radius vector
		return new Point(false,rotation3d(this.axis.radius,checkPoint(p)||this.point.radius,d))
	}
	Ray.prototype.travel = function (d,p=false) { // travel angle in degrees, from point as geolocation
		return this.travelRadias(radians(d),p)
	}
	
	function linesCrossingPoint2D(P1,V1,P2,V2) {
		var vd=VD(P2,P1)
		if (V1[0]!=0 && V2[0]!=0) {
			g1=V1[1]/V1[0]
			g2=V2[1]/V2[0]
			g=g1-g2
			if (g==0)
				if (vd[0]==0 || vd[1]/vd[0]!=g1)
					return true
				else
					return false
			P11=P1[1]+g1*(P2[0]-P1[0])
			dx=(P2[1]-P11)/g
			return [P2[0]+dx,P2[1]+g2*dx]
		} else if (V1[1]!=0 && V2[1]!=0) {
			g1=V1[0]/V1[1]
			g2=V2[0]/V2[1]
			g=g1-g2
			if (g==0)
				if (vd[1]==0 || vd[0]/vd[1]!=g1)
					return true
				else
					return false
			P10=P1[0]+g1*(P2[1]-P1[1])
			dy=(P2[0]-P10)/g
			return [P2[0]+g2*dy,P2[1]+dy]
		} else if (V1[0]!=0)
			return [P2[0],P1[1]]
		else
			return [P1[0],P2[1]]
	}

	return {
		RadiusVector:rv,
		Location:geo,
		Radians:radians,
		Degrees:degrees,
		Round:round,
		Ray:Ray,
		Point:Point,
		SphericalTwoOrtodromesIntersectionPoint: function(p1,p2,withDistance=false) {    			
			
			var R1=checkRay(p1)
			var R2=checkRay(p2)
			
			var v1=VP(R1.axis.radius,R2.axis.radius)
			if (interval(v1)<tol)
				return false
			v1=UV(v1)
			
			var v2=translate(v1,-1)
			
			var d11=distanceToCrossPoint(R1,v1)
			var d12=distanceToCrossPoint(R1,v2)
			var d21=distanceToCrossPoint(R2,v1)
			var d22=distanceToCrossPoint(R2,v2)
			
			if (d11+d21-d12-d22>tol || (Math.abs(d11+d21-d12-d22)<tol && d11-d12>tol)) {
				v1=v2
				d11=d12
				d21=d22
			}

			var result = new Point(false,v1)
			
			if (withDistance==false)
				return result
			else
				return [result,[round(degrees(d11)),d11,R1],[round(degrees(d21)),d21,R2]]
		},
		DistanceTo: function(p1,p2,inDegrees=true,withRay=false) {
			var r1=checkPoint(p1)
			var r2=checkPoint(p2)
			var r=angleDiff3d(r1.radius,r2.radius)
			if (inDegrees==true)
				r[0]=degrees(r[0])
			if (withRay==true) {
				var ray
				if (interval([r1.radius[1],r1.radius[0]])<tol)
					ray=new Ray(r1,r2.location[1])
				else{
					var par=VP(r1.radius,[0,0,1],true)
					var azv=rotation3d(r1.radius,r[1],-Math.PI/2)
					var mer=VP(par,r1.radius,[0,0,1])
					var angle=angleDiff3d(azv,mer)
					if (interval(angle[1],r1.radius)>1)
						angle[0]=Math.PI*2-angle[0]
					ray=new Ray(r1,degrees(angle[0]))
				}
				ray.axis = new Point(false,r[1])
				return [r[0],ray]
			} else
				return r[0]

		},
		SphericalTwoLoxodromesIntersectionPoint: function (p1,p2,withDistance=false) {
			var r1=radians(p1[0][0])
			var r2=radians(p2[0][0])
			var rr1=radians(p1[1])
			var rr2=radians(p2[1])
			var V1=UV([Math.cos(rr1)/Math.cos(r1)*Math.PI/2,Math.sin(rr1)])
			var V2=UV([Math.cos(rr2)/Math.cos(r2)*Math.PI/2,Math.sin(rr2)])
			var P1=[Math.tan(r1)*90,p1[0][1]]
			var P2=[Math.tan(r2)*90,p2[0][1]]
			var R=linesCrossingPoint2D(P1,V1,P2,V2)
			var D1,D2
			if (R===true) {
				R=[90,0]
				D1=[90-p1[0][0],NaN]
				D2=[90-p2[0][0],NaN]
			} else if (R===false)
				return false
			else {
				R[0]=round(degrees(Math.atan(R[0]/90)))
				D1=[R[0]-p1[0][0],R[1]-p1[0][1]]
				D2=[R[0]-p2[0][0],R[1]-p2[0][1]]
				R[1]=round(R[1]%360)
			}
			if (withDistance==false)
				return R
			else
				return [R,D1,D2]
			
		}
	}
})()
