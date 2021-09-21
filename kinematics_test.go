package kinematics

import (
	"math/rand"
	"testing"

	"gonum.org/v1/gonum/mat"
)

func TestForwardKinematics(t *testing.T) {
	testThetas := JointAngles{10, 1, 1, 0, 0, 0}
	f := ForwardKinematics(testThetas, AR3DhParameters)
	switch {
	case f.Pos.X != -101.74590611879692:
		t.Errorf("Forward kinematics failed on f.Pos.X = %f", f.Pos.X)
	case f.Pos.Y != -65.96805988175777:
		t.Errorf("Forward kinematics failed on f.Pos.Y = %f", f.Pos.Y)
	case f.Pos.Z != -322.27756822304093:
		t.Errorf("Forward kinematics failed on f.Pos.Z = %f", f.Pos.Z)
	case f.Rot.X != 0.06040824945687102:
		t.Errorf("Forward kinematics failed on f.Rot.X = %f", f.Rot.X)
	case f.Rot.Y != -0.20421099379003957:
		t.Errorf("Forward kinematics failed on f.Rot.Y = %f", f.Rot.Y)
	case f.Rot.Z != 0.2771553334491873:
		t.Errorf("Forward kinematics failed on f.Rot.Z = %f", f.Rot.Z)
	case f.Rot.W != 0.9369277637862541:
		t.Errorf("Forward kinematics failed on f.Rot.W = %f", f.Rot.W)
	}
}

func TestInverseKinematics(t *testing.T) {
	thetasInit := JointAngles{0, 0, 0, 0, 0, 0}
	desiredEndEffector := Pose{
		Position{
			-91.72345062922584,
			386.93155027870745,
			382.30917872225154},
		Quaternion{
			W: 0.41903052745255764,
			X: 0.4007833787652043,
			Y: -0.021233218878182854,
			Z: 0.9086418268616911}}
	_, err := InverseKinematics(desiredEndEffector, AR3DhParameters, thetasInit)
	if err != nil {
		t.Errorf("Inverse Kinematics failed with error: %s", err)
	}

	// This case should fail because the X required is too large
	desiredEndEffector = Pose{
		Position{-91000000.72345062922584,
			386.93155027870745,
			382.30917872225154},
		Quaternion{
			W: 0.41903052745255764,
			X: 0.4007833787652043,
			Y: -0.021233218878182854,
			Z: 0.9086418268616911}}
	_, err = InverseKinematics(desiredEndEffector, AR3DhParameters, thetasInit)
	if err == nil {
		t.Errorf("Inverse Kinematics should have failed with large X")
	}
}

func TestMatrixToQuaterion(t *testing.T) {
	var q Quaternion

	// Test tr > 0
	accumulatortMat1 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	q = matrixToQuaterion(accumulatortMat1)
	switch {
	case q.W != 1:
		t.Errorf("Failed mat1 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat1 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat1 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat1 with q.Z = %f", q.Z)
	}

	// Test (accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1)) &&
	// (accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2))
	accumulatortMat2 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 0,
	})
	q = matrixToQuaterion(accumulatortMat2)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat2 with qw = %f", q.W)
	case q.X != 1:
		t.Errorf("Failed mat2 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat2 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat2 with q.Z = %f", q.Z)
	}

	// Test accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2)
	accumulatortMat3 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, -2, 0,
		0, 0, 0, 1,
	})
	q = matrixToQuaterion(accumulatortMat3)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat3 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat3 with q.X = %f", q.X)
	case q.Y != 1:
		t.Errorf("Failed mat3 with q.Y = %f", q.Y)
	case q.Z != 0:
		t.Errorf("Failed mat3 with q.Z = %f", q.Z)
	}

	// Test default
	accumulatortMat4 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, -2, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1,
	})
	q = matrixToQuaterion(accumulatortMat4)
	switch {
	case q.W != 0:
		t.Errorf("Failed mat4 with q.W = %f", q.W)
	case q.X != 0:
		t.Errorf("Failed mat4 with q.X = %f", q.X)
	case q.Y != 0:
		t.Errorf("Failed mat4 with q.Y = %f", q.Y)
	case q.Z != 1:
		t.Errorf("Failed mat4 with q.Z = %f", q.Z)
	}
}

func BenchmarkInverseKinematics(b *testing.B) {
	thetasInit := JointAngles{0, 0, 0, 0, 0, 0}
	randTheta := func() float64 {
		return 360 * rand.Float64()
	}
	for i := 0; i < b.N; i++ {
		randomSeed := JointAngles{randTheta(), randTheta(), randTheta(),
			randTheta(), randTheta(), randTheta()}
		desiredEndEffector := ForwardKinematics(randomSeed, AR3DhParameters)
		_, err := InverseKinematics(desiredEndEffector, AR3DhParameters, thetasInit)
		if err != nil {
			b.Errorf("Failed inverse kinematics benchmark with: %s\nSeed:"+
				" %f-%f-%f-%f-%f-%f", err, randomSeed.J1, randomSeed.J2,
				randomSeed.J3, randomSeed.J4,
				randomSeed.J5, randomSeed.J6)
		}
	}
}
