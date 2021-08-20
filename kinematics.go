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
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/optimize"
	"math"
	"math/rand"
)

// DhParameters stand for "Denavit-Hartenberg Parameters". These parameters
// define a robotic arm for input into forward or reverse kinematics.
type DhParameters struct {
	ThetaOffsets []float64
	AlphaValues  []float64
	AValues      []float64
	DValues      []float64
}

// XyzWxyz represents an Xyz Qw-Qx-Qy-Qz coordinate, where Qw-Qx-Qy-Qz are
// quaternion coordinates for the rotation of a given end effector.
type XyzWxyz struct {
	X  float64
	Y  float64
	Z  float64
	Qx float64
	Qy float64
	Qz float64
	Qw float64
}

// ForwardKinematics calculates the end effector XyzWxyz coordinates given
// joint angles and robotic arm parameters.
func ForwardKinematics(thetas []float64, dhParameters DhParameters) XyzWxyz {
	// First, setup variables. We use 4 variables - theta, alpha, a and d to calculate a matrix
	// which is then multiplied to an accumulator matrix.
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
	for jointIdx := 0; jointIdx < len(thetas); jointIdx++ {
		theta = thetas[jointIdx]
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

	// Now that we have the final accumulatorMatrix, lets figure out the euler angles.
	var output XyzWxyz
	output.X = accumulatortMat.At(0, 3)
	output.Y = accumulatortMat.At(1, 3)
	output.Z = accumulatortMat.At(2, 3)
	output.Qw, output.Qx, output.Qy, output.Qz = matrixToQuaterian(accumulatortMat)
	return output
}

// MaxInverseKinematicIteration is the max number of times InverseKinematics
// should try new seeds before failing. 50 is used here because it is
// approximately the number of iterations that will take 1s to compute.
var MaxInverseKinematicIteration int = 50

// InverseKinematics calculates joint angles to achieve an XyzWxyz end effector
// position given the desired XyzWxyz coordinates and the robotic arm
// parameters.
func InverseKinematics(desiredEndEffector XyzWxyz, dhParameters DhParameters) ([]float64, error) {
	thetasInit := []float64{0, 0, 0, 0, 0, 0}
	// Initialize an objective function for the optimization problem
	objectiveFunction := func(s []float64) float64 {
		currentEndEffector := ForwardKinematics(s, dhParameters)

		// Get XYZ offsets
		xOffset := desiredEndEffector.X - currentEndEffector.X
		yOffset := desiredEndEffector.Y - currentEndEffector.Y
		zOffset := desiredEndEffector.Z - currentEndEffector.Z

		// Get rotational offsets. Essentially, do this in Golang (from python): np.arccos(np.clip(2*(np.dot(target_quat, source_quat)**2) - 1, -1, 1))
		dotOffset := (desiredEndEffector.Qw * currentEndEffector.Qw) + (desiredEndEffector.Qx * currentEndEffector.Qx) + (desiredEndEffector.Qy * currentEndEffector.Qy) + (desiredEndEffector.Qz * currentEndEffector.Qz)
		dotOffset = (2*(dotOffset*dotOffset) - 1)
		if dotOffset > 1 {
			dotOffset = 1
		}
		rotationalOffset := math.Acos(dotOffset)

		// Get the error vector
		errorVector := ((xOffset * xOffset) + (yOffset * yOffset) + (zOffset * zOffset) + (rotationalOffset * rotationalOffset)) * 0.25

		return errorVector
	}
	// Setup problem and method for solving
	problem := optimize.Problem{Func: objectiveFunction}

	// Solve
	result, err := optimize.Minimize(problem, thetasInit, nil, nil)
	if err != nil {
		return []float64{}, err
	}
	f := result.Location.F

	// If the results aren't up to spec, queue up another theta seed and test again.
	// We arbitrarily choose 10e-6 because that is small enough that the errors do not matter.
	for i := 0; f > 0.000001; i++ {
		// Get a random seed
		randTheta := func() float64 {
			return 360 * rand.Float64()
		}
		randomSeed := []float64{randTheta(), randTheta(), randTheta(), randTheta(), randTheta(), randTheta()}

		// Solve
		result, err := optimize.Minimize(problem, randomSeed, nil, nil)
		if err != nil {
			return []float64{}, err
		}
		f = result.Location.F
		if i == MaxInverseKinematicIteration {
			return []float64{}, errors.New("Desired position out of range of the robotic arm.")
		}
	}
	return result.Location.X, nil
}

// matrixToQuaterian converts a rotation matrix to a quaterian. This code has
// been tested in all cases vs the python implementation with scipy rotation
// and works properly.
func matrixToQuaterian(accumulatortMat *mat.Dense) (float64, float64, float64, float64) {
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	var qw float64
	var qx float64
	var qy float64
	var qz float64
	var tr float64
	var s float64
	tr = accumulatortMat.At(0, 0) + accumulatortMat.At(1, 1) + accumulatortMat.At(2, 2)
	switch {
	case tr > 0:
		s = math.Sqrt(tr+1.0) * 2
		qw = 0.25 * s
		qx = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		qy = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		qz = (accumulatortMat.At(1, 0) - accumulatortMat.At(0, 1)) / s
	case accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1) && accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2):
		s = math.Sqrt(1.0+accumulatortMat.At(0, 0)-accumulatortMat.At(1, 1)-accumulatortMat.At(2, 2)) * 2
		qw = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		qx = 0.25 * s
		qy = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		qz = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
	case accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2):
		s = math.Sqrt(1.0+accumulatortMat.At(1, 1)-accumulatortMat.At(0, 0)-accumulatortMat.At(2, 2)) * 2
		qw = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		qx = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		qy = 0.25 * s
		qz = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
	default:
		s = math.Sqrt(1.0+accumulatortMat.At(2, 2)-accumulatortMat.At(0, 0)-accumulatortMat.At(1, 1)) * 2
		qw = (accumulatortMat.At(0, 1) - accumulatortMat.At(1, 0))
		qx = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
		qy = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
		qz = 0.25 * s
	}
	return qw, qx, qy, qz
}
