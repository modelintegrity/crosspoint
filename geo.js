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
		var u,s=VP(vo,vr,true,true)
		if (s==0)
			u=VP(vo,[vo[2],vo[0],vo[1]],true)
        var a=Math.asin(Math.max(-1,Math.min(s,1)))
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
            V=V1
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
	
	function linesCrossingPoint2DExtHelper1(V_,P,i,isRay) {
		var V
		if (i<tol) {
			V=[0,0]
			isRay=true
			//fallback for vector
			if (isNaN(P[0])) {
				isRay=false
				V[0]=i=1
			} else if (isNaN(P[1])) {
				isRay=false
				V[1]=i=1
			}	
		} else if (isRay)
			V=UV(V_)
		else
			V=V_
		return [V,i]
	}
	
	function linesCrossingPoint2DExt(P1,V1_,P2,V2_,isRay1=false,isRay2=false,middlePointFallback=false) { //exceptions: true - points are on the same line (rays overlaps) - an infinite number of crosspoints, null - lines are parallel and not in the same line (rays look in same direction) - the crosspoint somewhere in the infinity, false - rays do not intersect or vectors are invalid
		if (interval(P1,P2)<tol) // same normal points, automatically returns the middle
			return VA(P1,P2)
		
		if ((isNaN(P1[0]) && isNaN(P1[1])) || (isNaN(P2[0]) && isNaN(P2[1]))) // no sufficient data for one of points
			return undefined
		
		var i1,i2,V1,V2
		[V1,i1]=linesCrossingPoint2DExtHelper1(V1_,P1,interval(V1_),isRay1);
		[V2,i2]=linesCrossingPoint2DExtHelper1(V2_,P2,interval(V1_),isRay2)
				
		if (i1<tol && i2<tol) // not same points but same time no vectors to search for each other
			return false

		if ((isNaN(P1[0]) && Math.abs(V1[1])>tol) || (isNaN(P1[1]) && Math.abs(V1[0])>tol) || (isNaN(P2[0]) && Math.abs(V2[1])>tol) || (isNaN(P2[1]) && Math.abs(V2[0])>tol)) // uncertainty of lines definition
			return undefined
			
		return linesCrossingPoint2D(P1,V1,P2,V2,isRay1,isRay2,middlePointFallback,i1,i2) 
	}

	function linesCrossingPoint2D(P1,V1,P2,V2,isRay1=false,isRay2=false,middlePointFallback=false,i1=1,i2=1) { //exceptions: true - points are on the same line (rays overlaps) - an infinite number of crosspoints, null - lines are parallel and not in the same line (rays look in same direction) - the crosspoint somewhere in the infinity, false - rays do not intersect or vectors are invalid
		
		var d,x,y
		
		if (i1>tol && i2>tol) {
			var	c1=(isNaN(P1[0])?0:P1[0])*V1[1]-(isNaN(P1[1])?0:P1[1])*V1[0]
			var	c2=(isNaN(P2[0])?0:P2[0])*V2[1]-(isNaN(P2[1])?0:P2[1])*V2[0]
			d=V2[1]*V1[0]-V1[1]*V2[0]
			x=V1[0]*c2-V2[0]*c1
			y=V1[1]*c2-V2[1]*c1
		} else
			d=x=y=0
	
		if (Math.abs(d)>tol) { // a real intersection of two separate lines is determined (a general case)
			var R=[x/d,y/d]
			if ((!isRay1 || interval(UV(VD(R,P1)),V1)<1 || interval(R,P1)<tol || isNaN(P1[0]) || isNaN(P1[1])) && 
				(!isRay2 || interval(UV(VD(R,P2)),V2)<1 || interval(R,P2)<tol || isNaN(P2[0]) || isNaN(P2[1]))) // no set lines direction or rays in a collision direction (a general case)
				return R
			else // rays are in avoidance directions, so actually there is no rays' intersection point
				return false
		} else if (Math.abs(x)<tol && Math.abs(y)<tol) { // initial points are on one and the same line or there is only one line available
			
			var v12=UV(VD(P1,P2))
			var v21=UV(VD(P2,P1))
			var int2=interval(v12,V2)
			var int1=interval(v21,V1)			
					
			if ((i1<tol && (interval(v12,UV(V2))>tol || interval(v21,UV(V2))>tol)) || (i2<tol && (interval(v21,UV(V1))>tol || interval(v12,UV(V1))>tol))) // the only availalabe line does not cross both points 
				return false
			else if (int2>tol && int1>tol && isRay1 && isRay2) // opposite direction of rays with no overlapping, rays look opposite of other points direction
				return false
			else if ((isNaN(P1[0]) || isNaN(P1[1])) && !isNaN(P2[0]) && !isNaN(P2[1])) // condition to choose the only real point available
				return (i2<tol || middlePointFallback)?P2:true
			else if ((isNaN(P2[0]) || isNaN(P2[1])) && !isNaN(P1[0]) && !isNaN(P1[1])) // condition to choose the only real point available
				return (i1<tol || middlePointFallback)?P1:true
			else if (!middlePointFallback) // there is overlapping with infinite number of common points, but no need to choose one (possible in the middle)
				return true
			else if (isNaN(P1[0]) || isNaN(P1[1]) || isNaN(P2[0]) || isNaN(P2[1])) // still no points available due to no sufficient data
				return true
			else if ((!isRay1 || int1<tol) && (!isRay2 || int2<tol)) // condition to choose a certain point in a middle, as for both points: no line direction or ray direction looks towards the other point 
				return VA(P1,P2)
			else if (isRay2 && int2>1) // condition to choose the second point as the second ray looks outwards of the first point and the second point is the closest to the middle out of all common points 
				return P2
			else if (isRay1 && int1>1)// condition to choose the first point as the first ray looks outwards of the second point and the first point is the closest to the middle out of all common points 
				return P1
			else   //should not reach there but if it does then no determined point
				return true
		} else // initial points are on two separate parallel lines
			if (!isRay1 || !isRay2 || interval(V1,V2)<1) // no line direction or same rays direction 
				return null
			else // opposite rays direction
				return false
	}
	
	function rv(p) {
		var lat=radians(p[0])
        var lng=radians(p[1])
        var cos=Math.cos(lat)
		var r=[Math.cos(lng)*cos,Math.sin(lng)*cos,Math.sin(lat)]
		if (Math.abs(Math.abs(r[2])-1)<tol)
			r=[0,0,Math.round(r[2])]
        return r
	}
	
	function geo(r) {
        var lat=degrees(Math.asin(r[2])),lng
		if (Math.abs(Math.abs(lat)-90)<tol) {
            lat=Math.round(lat)
			lng=0
        } else {
            var r_=UV([r[0],r[1],0])
            lng=degrees(angle(r_[0],r_[1]))
			if (lng>180)
				lng=-360+lng
		}
		return [lat,lng]
	}
	
	function ortDistanceToCrossPoint(R,v) {
		var d=angleDiff3d(R.point.radius,v)
		if (interval(d[1],R.axis.radius)>1)
			d[0]=Math.PI*2-d[0]
		if (Math.abs(d[0]-Math.PI*2)<tol)
			d[0]=0
		return d[0]
	}
	
	function loxDistanceToCrossPoint(R,P){
		if (Math.abs(Math.abs(R.point.location[0])-90)<tol) {
			i=Math.abs(P.location[0]-R.point.location[0])
		} else {
			var i=interval(P.radius,R.point.radius)
			if (i>tol || Math.abs(Math.abs(P.location[0])-90)<tol) {
				var cos=Math.abs(Math.cos(R.azimuthRad))
				if (cos<tol)
					i=Math.abs((P.mercatorLocation[1]-R.point.mercatorLocation[1])/Math.sin(R.azimuthRad)*Math.cos((P.locationRad[0]+R.point.locationRad[0])/2))			
				else 
					i=Math.abs(P.location[0]-R.point.location[0])/cos
			} else 
				i=degrees(i)
		}
		return i
	}
	
	var Point = function(coordinates) {
		var len=coordinates.length
		if (len==2) {
			this.radius=rv(coordinates)
			this.location=[coordinates[0],coordinates[1]%360]
			if (this.location[1]<0)
				this.location[1]+=360
		} else if (len==3) {
			this.radius=[coordinates[0],coordinates[1],coordinates[2]]
			this.location=geo(coordinates)			
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
		var lng=(R[1]+180)%360-180
		if (lng<-180)
			lng+=360
		P = new Point([degrees(Math.atan(R[0]/90)),lng])
		P.mercatorLocation=[R[0],R[1]] 
		return P
	}
	Point.prototype.mercator=function(){
		if (Math.abs(Math.abs(this.location[0])-90)<tol)
			this.mercatorLocation=[NaN,this.location[1]]
		else
			this.mercatorLocation=[Math.tan(this.locationRad[0])*90,this.location[1]]
		return this
	}
	
	var Ray = function(p,a) {
		this.point=Point.check(p)
		this.azimuth=a%360
		if (this.azimuth<0)
			this.azimuth+=360
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
		this.point.mercator()
		if (Math.abs(this.point.location[0]-90)<tol) {
			this.mercatorVector=[-1,0]
			this.point.mercatorLocation[1]=this.azimuth
		} else if (Math.abs(this.point.location[0]+90)<tol) {
			this.mercatorVector=[1,0]
			this.point.mercatorLocation[1]=this.azimuth
		} else
			this.mercatorVector=UV([Math.cos(this.azimuthRad)/Math.cos(this.point.locationRad[0])*Math.PI/2,Math.sin(this.azimuthRad)])
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
		this.axis=new Point(rotation3d(this.point.radius,par,a))
		return this
	}
	Ray.prototype.travelRadians = function (d,p=false) { //travel angle in radians, from point as radius vector
		return new Point(rotation3d(this.axis.radius,Point.check(p).radius||this.point.radius,d))
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
							
			var d11=ortDistanceToCrossPoint(R1,v1)
			var d12=ortDistanceToCrossPoint(R1,v2)
			var d21=ortDistanceToCrossPoint(R2,v1)
			var d22=ortDistanceToCrossPoint(R2,v2)
				
			if (d11+d21-d12-d22>tol || (Math.abs(d11+d21-d12-d22)<tol && d11-d12>tol)) {
				v1=v2
				d11=d12
				d21=d22
			}

			var result = new Point(v1)
			
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
				ray.axis = new Point(r[1])
				return [r[0],ray]
			} else
				return r[0]
		},
		SphericalTwoLoxodromesIntersectionPoint: function (p1,p2,withDistance=false) {
			
			var R1=Ray.check(p1).mercator()
			var R2=Ray.check(p2).mercator()
			
			var shift=0
			if (interval(R1.point.radius,R2.point.radius)<tol) // same points catch
				P=new Point(UV(VA(R1.point.radius,R2.point.radius)))
			else {
				var P=linesCrossingPoint2DExt(R1.point.mercatorLocation,R1.mercatorVector,R2.point.mercatorLocation,R2.mercatorVector,true,true,true)
				if (P===false) { //the first attempt is not sucessfull
					shift=360 
					R2.point.mercatorLocation[1]+=shift //thus shifting the second point one loop forward and repeating
					P=linesCrossingPoint2DExt(R1.point.mercatorLocation,R1.mercatorVector,R2.point.mercatorLocation,R2.mercatorVector,true,true,true)
					if (P===false) { //the second attempt is not sucessfull
						R2.point.mercatorLocation[1]-=shift
						shift=-360
						R2.point.mercatorLocation[1]+=shift //thus shifting the second point one loop backward and repeating
						P=linesCrossingPoint2DExt(R1.point.mercatorLocation,R1.mercatorVector,R2.point.mercatorLocation,R2.mercatorVector,true,true,true)
					}
				} else if (P===null) { // rays are two parallels heading same direction
					if (Math.abs(Math.cos(R1.azimuthRad))<tol) // and it turns out they are just two geographical parallels, thus not intersecting
						return false
					P=new Point([R1.mercatorVector[0]>0?90:-90,0]) // and reach a the same pole
				} else if (P===undefined) // something goes wrong, but should not be a case in the case
					return false
					 
				if (P===false || P===true) // true - loxodromes launched from opposite poles towards the same longitide, false - loxodromes launched from opposite poles towards different longitide, or there is no intersection at all  
					if (Math.abs(Math.abs(R1.point.location[0])-90)<tol) // the first initial point is  a pole
						P=new Point(R1.point.location)
					else if (Math.abs(Math.abs(R2.point.location[0])-90)<tol) // the second initial point is a pole
						P=new Point(R2.point.location)
					else
						return false // rays do not intersect
			}
			
			if (!(P instanceof Point)) // normal
				P=Point.fromMercator(P)
			else				
				P.mercator()
			
			if (Math.abs(Math.abs(P.location[0])-90)<tol) // cross point is a pokem, so no sense of longitude path as it makes infinite number of loop reaching the pole.
				P.mercatorLocation[1]=NaN
				
			if (!withDistance)
				return P
			else {
				var D1=[round(loxDistanceToCrossPoint(R1,P)),Math.abs(R1.mercatorVector[1])<tol?0:P.mercatorLocation[1]-R1.point.mercatorLocation[1]]
				var D2=[round(loxDistanceToCrossPoint(R2,P)),Math.abs(R2.mercatorVector[1])<tol?0:P.mercatorLocation[1]-R2.point.mercatorLocation[1]]
				R2.point.mercatorLocation[1]-=shift
				return [P,[D1,R1],[D2,R2]]
			}
		}
	}
})()
