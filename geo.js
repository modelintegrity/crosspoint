



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
	
    function VA(V1,V2=false){
        var V
		if (V2)
            V=[V1,V2]
        else
            V=[V1]
        var p=V.length
        var l=V[0].length
        var r=[]
        for (var i=0;i<l;i++) {
			r.push(0)
			for (var j=0;j<p;j++)
				r[i]+=V[j][i]/p
		}
        return r
	}
	
	function VD(V1,V2) {
        var l=V1.length
        var r=[]
        for (var i=0;i<l;i++)
            r.push(V1[i]-V2[i])
        return r
	}
	
	function linesCrossingPoint2D(P1,V1_,P2,V2_,isRay=false,middlePointFallback=false) { //exceptions: null - points are on the same line (rays overlaps) - an infinite number of crosspoints, true - lines are parallel and not in the same line (rays look in same direction) - the crosspoint somewhere in the infinity, false - rays do not intersect or vectors are invalid
		if (interval(P1,P2)<tol)
			return VA(P1,P2)
		if (interval(V1_)==0 || interval(V2_)==0)
			return false
		var V1,V2
		if (!isRay) {
			V1=V1_
			V2=V2_
		} else {
			V1=UV(V1_)
			V2=UV(V2_)
		}
		var a1=-V1[1]
		var b1=V1[0]
		var a2=-V2[1]
		var b2=V2[0]
		var c1,c2
		if (Math.abs(a1)<Math.abs(b1))
			c1=-P1[0]*a1-P1[1]*b1
		else
			c1=-P1[1]*b1-P1[0]*a1
		if (Math.abs(a2)<Math.abs(b2))
			c2=-P2[0]*a2-P2[1]*b2
		else
			c2=-P2[1]*b2-P2[0]*a2
		var dem=a1*b2-a2*b1
		var x=b1*c2-b2*c1
		var y=a2*c1-a1*c2
		if (Math.abs(dem)>tol) {
			var R=[x/dem,y/dem]
			if (!isRay || (interval(UV(VD(R,P1)),V1)<1 && interval(UV(VD(R,P2)),V2)<1))
				return R
			else
				return false
		} else if (Math.abs(x)<tol && Math.abs(y)<tol)
			if (!isRay)
				if (middlePointFallback)
					return VA(P1,P2)
				else
					return null
			else {
				var int2=interval(UV(VD(P1,P2)),V2)
				var int1=interval(UV(VD(P2,P1)),V1)
				if (int2>1 && int1>1)
					return false
				else if (middlePointFallback)
					if (int1>1)
						return P1
					else if (int2>1)
						return P2
					else
						return VA(P1,P2)
				else
					return null
			}
		else 
			if (!isRay || interval(V1,V2)<1)
				return true
			else 
				return false
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
	
	var Point = function(location,radius) {
		if (location!=false) {
			this.radius=rv(location)
			this.location=[location[0],location[1]%360]
			if (this.location[1]<0)
				this.location[1]+=360
		} else if (radius!=false) {
			this.radius=radius
			this.location=geo(radius)			
		}
		this.locationRad=[radians(this.location[0]),radians(this.location[1])]
		this.locationRounded=[round(this.location[0]),round(this.location[1])]
	}
	Point.check = function(p) {
		var P
		if (p instanceof Array)
			P=new Point(p)
		else
			P=p
		return P
	}
	Point.fromMercator = function(R) {
		var lng=round(R[1]%360)
		if (lng<0)
			lng+=360
		P = new Point([round(degrees(Math.atan(R[0]/90))),lng])
		P.mercatorLocation=[R[0],R[1]] 
		return P
	}
	Point.prototype.mercator=function(){
		this.mercatorLocation=[Math.tan(this.locationRad[0])*90,this.location[1]]
		return this
	}
	
	var Ray = function(p,a) {
		this.point=Point.check(p)
		this.azimuth=a
		this.azimuthRad=radians(a)
	}
	Ray.check =function(p) {
		var R
		if (p instanceof Array)
			R=new Ray(p[0],p[1])
		else
			R=p
		return R
	}
	Ray.prototype.mercator=function() {
		this.mercatorVector=UV([Math.cos(this.azimuthRad)/Math.cos(this.point.locationRad[0])*Math.PI/2,Math.sin(this.azimuthRad)])
		this.point.mercator()
		return this
	}
	Ray.prototype.axis = function() {
		var par,a,k=interval([this.point.radius[0],this.point.radius[1]])
		if (k>tol) {
            par=VP(this.point.radius,[0,0,1],true)
			a=-this.azimuthRad
        } else
			if (this.point.radius[2]>0) {
				par=[0,1,0]
	            a=this.azimuthRad
			} else {
				par=[0,-1,0]
	            a=-this.azimuthRad
			}
		this.axis=new Point(false,rotation3d(this.point.radius,par,a))
		return this
	}
	Ray.prototype.travelRadians = function (d,p=false) { //travel angle in radians, from point as radius vector
		return new Point(false,rotation3d(this.axis.radius,Point.check(p).radius||this.point.radius,d))
	}
	Ray.prototype.travel = function (d,p=false) { // travel angle in degrees, from point as geolocation
		return this.travelRadians(radians(d),p)
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
			
			var R1=Ray.check(p1).axis()
			var R2=Ray.check(p2).axis()
			
			var v1=VP(R1.axis.radius,R2.axis.radius),v2
				
			if (interval(v1)<tol) {
				if (interval(R1.axis.radius,R2.axis.radius)<1) {
					v1=R1.point.radius
					v2=R2.point.radius
				} else {
					v1=VA(R1.point.radius,R2.point.radius)
					if (interval(v1)>tol)
						v1=UV(v1)
					else
						v1=VA(VP(R1.point.radius,R1.axis.radius),VP(R2.point.radius,R2.axis.radius))
				}
			} else
				v1=UV(v1)
		
			if (!v2)
				v2=translate(v1,-1)
							
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
		OrtodromeDistanceTo: function(p1,p2,inDegrees=true,withRay=false) {
			
			var r1=Point.check(p1)
			var r2=Point.check(p2)
			
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
			
			var R1=Ray.check(p1).mercator()
			var R2=Ray.check(p2).mercator()
			var shift=0
			R2.point.mercatorLocation[1]+=shift
			var P=linesCrossingPoint2D(R1.point.mercatorLocation,R1.mercatorVector,R2.point.mercatorLocation,R2.mercatorVector,true,true)
			if (P===false) {
				R2.point.mercatorLocation[1]-=shift
				shift=360
				R2.point.mercatorLocation[1]+=shift
				P=linesCrossingPoint2D(R1.point.mercatorLocation,R1.mercatorVector,R2.point.mercatorLocation,R2.mercatorVector,true,true)
				if (P===false) {
					R2.point.mercatorLocation[1]-=shift
					shift=-360
					R2.point.mercatorLocation[1]+=shift
					P=linesCrossingPoint2D(R1.point.mercatorLocation,R1.mercatorVector,R2.point.mercatorLocation,R2.mercatorVector,true,true)
					if (P===false)
						return false
				}
			}
			
			if (P===true) {
				P=new Point([R1.mercatorVector[0]>0?90:-90,0])
				P.mercatorLocation=[NaN,NaN]
			} else
				P=Point.fromMercator(P)
			
			if (!withDistance)
				return P
			else {
				var i1=interval(P.locationRad,R1.point.locationRad)
				if (i1>tol){
					var latdiff1=Math.abs(P.locationRad[0]-R1.point.locationRad[0])
					var cos1=Math.abs(Math.cos(R1.azimuthRad))
					if ((Math.abs(P.location[0])==90 && cos1>tol) || (latdiff1>0.05 && cos1>0.05))
						i1=latdiff1/cos1
					else if (Math.abs(P.location[0])==90)
						i1=NaN
					else
						i1=Math.abs((P.locationRad[1]-R1.point.locationRad[1])/Math.sin(R1.azimuthRad)*Math.cos((P.locationRad[0]+R1.point.locationRad[0])/2))
				}
				var i2=interval(P.locationRad,R2.point.locationRad)
				if (i2>tol){
					var latdiff2=Math.abs(P.locationRad[0]-R2.point.locationRad[0])
					var cos2=Math.abs(Math.cos(R2.azimuthRad))
					if ((Math.abs(P.location[0])==90 && cos2>tol) || (latdiff2>0.05 && cos2>0.05))
						i2=latdiff2/cos2
					else if (Math.abs(P.location[0])==90)
						i2=NaN
					else
						i2=Math.abs((P.locationRad[1]-R2.point.locationRad[1])/Math.sin(R2.azimuthRad)*Math.cos((P.locationRad[0]+R2.point.locationRad[0])/2))
				}
				var D1=[round(degrees(i1)),Math.abs(R1.mercatorVector[1])<tol?0:P.mercatorLocation[1]-R1.point.mercatorLocation[1]]
				var D2=[round(degrees(i2)),Math.abs(R2.mercatorVector[1])<tol?0:P.mercatorLocation[1]-R2.point.mercatorLocation[1]]
				R2.point.mercatorLocation[1]-=shift
				return [P,[D1,R1],[D2,R2]]
			}
		}
	}
})()
