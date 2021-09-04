/*
Package kinematics calculates forward and inverse kinematics for robotic arm
systems.

Forward kinematics takes joint angles and returns an XyzWxyz coordinate of the
end effector. Inverse kinematics takes an XyzWxyz coordinate and returns joint
angles that move the end effector to that coordinate.

 ForwardKinematics (joint angles	-> xyzwxyz)
 InverseKinematics (xyzwxyz	-> joint angles)

Each function also requires a Denavit-Hartenberg Parameter set for the arm of
interest. This package provides defaults for the following arm systems:

 AR2/AR3
*/
package kinematics

import (
	"errors"
	"math"
	"math/rand"

	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/optimize"
)

// DhParameters stand for "Denavit-Hartenberg Parameters". These parameters
// define a robotic arm for input into forward or reverse kinematics.
type DhParameters struct {
	ThetaOffsets [6]float64
	AlphaValues  [6]float64
	AValues      [6]float64
	DValues      [6]float64
}

// JointAngles represents angles of the joints.
type JointAngles struct {
	J1 float64
	J2 float64
	J3 float64
	J4 float64
	J5 float64
	J6 float64
}

func (st *JointAngles) toFloat() []float64 {
	return []float64{st.J1, st.J2, st.J3, st.J4, st.J5, st.J6}
}

// Quaternion represents a quaternion, which can be used to represent rotations
// in 3 space.
type Quaternion struct {
	Qx float64
	Qy float64
	Qz float64
	Qw float64
}

// Position represents a position in 3D cartesian space.
type Position struct {
	X float64
	Y float64
	Z float64
}

// Pose represents a position and rotation, where Position is the translational
// component, and Rot is the quaternion representing the rotation.
type Pose struct {
	Pos Position
	Rot Quaternion
}

// ForwardKinematics calculates the end effector Pose coordinates given
// joint angles and robotic arm parameters.
func ForwardKinematics(thetas JointAngles, dhParameters DhParameters) Pose {
	// First, setup variables. We use 4 variables - theta, alpha, a and d to
	// calculate a matrix which is then multiplied to an accumulator matrix.
	thetaArray := []float64{thetas.J1, thetas.J2, thetas.J3,
		thetas.J4, thetas.J5, thetas.J6}
	var theta float64
	var alpha float64
	var a float64
	var d float64
	// Setup accumulator matrix - an identity matrix.
	accumulatortMat := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	// Iterate through each joint and built a new
	// matrix, multiplying it against the accumulator.
	for jointIdx := 0; jointIdx < 6; jointIdx++ {
		theta = thetaArray[jointIdx]
		theta = theta + dhParameters.ThetaOffsets[jointIdx]
		alpha = dhParameters.AlphaValues[jointIdx]
		a = dhParameters.AValues[jointIdx]
		d = dhParameters.DValues[jointIdx]
		tMat := mat.NewDense(4, 4, []float64{
			// First row
			math.Cos(theta),
			-math.Sin(theta) * math.Cos(alpha),
			math.Sin(theta) * math.Sin(alpha),
			a * math.Cos(theta),
			// Second row
			math.Sin(theta),
			math.Cos(theta) * math.Cos(alpha),
			-math.Cos(theta) * math.Sin(alpha),
			a * math.Sin(theta),
			// Third row
			0,
			math.Sin(alpha),
			math.Cos(alpha),
			d,
			// Forth row
			0,
			0,
			0,
			1,
		})
		// Multiply tMat against accumulatortMat
		x := mat.NewDense(4, 4, nil)
		x.Mul(accumulatortMat, tMat)
		accumulatortMat = x
	}

	// Now that we have the final accumulatorMatrix, lets figure out the
	// quaternions angles.
	var output Pose
	output.Pos.X = accumulatortMat.At(0, 3)
	output.Pos.Y = accumulatortMat.At(1, 3)
	output.Pos.Z = accumulatortMat.At(2, 3)
	output.Rot = matrixToQuaterion(accumulatortMat)
	return output
}

// MaxInverseKinematicIteration is the max number of times InverseKinematics
// should try new seeds before failing. 50 is used here because it is
// approximately the number of iterations that will take 1 second to compute.
var MaxInverseKinematicIteration int = 50

// InverseKinematics calculates joint angles to achieve an end effector Pose
// given the desired Pose coordinates, the robotic DH parameters and a starting
// set of joint angles
func InverseKinematics(desiredEndEffector Pose, dhParameters DhParameters,
	thetasInit JointAngles) (JointAngles, error) {
	// Initialize an objective function for the optimization problem
	objectiveFunction := func(s []float64) float64 {
		jointAnglesTest := JointAngles{s[0], s[1], s[2], s[3], s[4], s[5]}
		currentEndEffector := ForwardKinematics(jointAnglesTest, dhParameters)

		// Get XYZ offsets
		xOffset := desiredEndEffector.Pos.X - currentEndEffector.Pos.X
		yOffset := desiredEndEffector.Pos.Y - currentEndEffector.Pos.Y
		zOffset := desiredEndEffector.Pos.Z - currentEndEffector.Pos.Z

		// Get rotational offsets. Essentially, do this in Golang (from python):
		// np.arccos(np.clip(2*(np.dot(target_quat, source_quat)**2) - 1, -1, 1))
		dotOffset := (desiredEndEffector.Rot.Qw * currentEndEffector.Rot.Qw) +
			(desiredEndEffector.Rot.Qx * currentEndEffector.Rot.Qx) +
			(desiredEndEffector.Rot.Qy * currentEndEffector.Rot.Qy) +
			(desiredEndEffector.Rot.Qz * currentEndEffector.Rot.Qz)
		dotOffset = (2*(dotOffset*dotOffset) - 1)
		if dotOffset > 1 {
			dotOffset = 1
		}
		rotationalOffset := math.Acos(dotOffset)

		// Get the error vector
		errorVector := ((xOffset * xOffset) +
			(yOffset * yOffset) +
			(zOffset * zOffset) +
			(rotationalOffset * rotationalOffset)) * 0.25

		return errorVector
	}
	// Setup problem and method for solving
	problem := optimize.Problem{Func: objectiveFunction}

	// Solve
	result, err := optimize.Minimize(problem, thetasInit.toFloat(), nil, nil)
	if err != nil {
		return JointAngles{}, err
	}
	f := result.Location.F

	// If the results aren't up to spec, queue up another theta seed and test
	// again. We arbitrarily choose 1e-6 because that is small enough that the
	// errors do not matter.
	for i := 0; f > 1e-6; i++ {
		// Get a random seed between -pi and pi in radians. 2pi == -pi, so we
		// use this simplify things.
		randTheta := func() float64 {
			return 2 * math.Pi * rand.Float64()
		}
		randomSeed := JointAngles{randTheta(), randTheta(), randTheta(),
			randTheta(), randTheta(), randTheta()}

		// Solve
		result, err := optimize.Minimize(problem, randomSeed.toFloat(), nil, nil)
		if err != nil {
			return JointAngles{}, err
		}
		f = result.Location.F
		if i == MaxInverseKinematicIteration {
			return JointAngles{}, errors.New("desired position out of range" +
				" of the robotic arm")
		}
	}
	r := result.Location.X
	return JointAngles{r[0], r[1], r[2], r[3], r[4], r[5]}, nil
}

// matrixToQuaterion converts a rotation matrix to a quaterian. This code has
// been tested in all cases vs the python implementation with scipy rotation
// and works properly.
func matrixToQuaterion(accumulatortMat *mat.Dense) Quaternion {
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	var qw float64
	var qx float64
	var qy float64
	var qz float64
	var tr float64
	var s float64
	tr = accumulatortMat.At(0, 0) + accumulatortMat.At(1, 1) +
		accumulatortMat.At(2, 2)
	switch {
	case tr > 0:
		s = math.Sqrt(tr+1.0) * 2
		qw = 0.25 * s
		qx = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		qy = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		qz = (accumulatortMat.At(1, 0) - accumulatortMat.At(0, 1)) / s
	case accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1) &&
		accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2):
		s = math.Sqrt(1.0+accumulatortMat.At(0, 0)-accumulatortMat.At(1, 1)-
			accumulatortMat.At(2, 2)) * 2
		qw = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		qx = 0.25 * s
		qy = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		qz = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
	case accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2):
		s = math.Sqrt(1.0+accumulatortMat.At(1, 1)-accumulatortMat.At(0, 0)-
			accumulatortMat.At(2, 2)) * 2
		qw = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		qx = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		qy = 0.25 * s
		qz = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
	default:
		s = math.Sqrt(1.0+accumulatortMat.At(2, 2)-accumulatortMat.At(0, 0)-
			accumulatortMat.At(1, 1)) * 2
		qw = (accumulatortMat.At(0, 1) - accumulatortMat.At(1, 0))
		qx = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
		qy = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
		qz = 0.25 * s
	}
	return Quaternion{qx, qy, qz, qw}
}
