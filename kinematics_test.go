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
		t.Errorf("Forward kinematics failed on f.X = %f", f.Pos.X)
	case f.Pos.Y != -65.96805988175777:
		t.Errorf("Forward kinematics failed on f.Y = %f", f.Pos.Y)
	case f.Pos.Z != -322.27756822304093:
		t.Errorf("Forward kinematics failed on f.Z = %f", f.Pos.Z)
	case f.Rot.Qx != 0.06040824945687102:
		t.Errorf("Forward kinematics failed on f.Qx = %f", f.Rot.Qx)
	case f.Rot.Qy != -0.20421099379003957:
		t.Errorf("Forward kinematics failed on f.Qy = %f", f.Rot.Qy)
	case f.Rot.Qz != 0.2771553334491873:
		t.Errorf("Forward kinematics failed on f.Qz = %f", f.Rot.Qz)
	case f.Rot.Qw != 0.9369277637862541:
		t.Errorf("Forward kinematics failed on f.Qw = %f", f.Rot.Qw)
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
			0.4007833787652043,
			-0.021233218878182854,
			0.9086418268616911,
			0.41903052745255764}}
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
			0.4007833787652043,
			-0.021233218878182854,
			0.9086418268616911,
			0.41903052745255764}}
	_, err = InverseKinematics(desiredEndEffector, AR3DhParameters, thetasInit)
	if err == nil {
		t.Errorf("Inverse Kinematics should have failed with large X")
	}
}

func TestMatrixToQuaterian(t *testing.T) {
	var quat Quaternion

	// Test tr > 0
	accumulatortMat1 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	quat = matrixToQuaterian(accumulatortMat1)
	switch {
	case quat.Qw != 1:
		t.Errorf("Failed mat1 with qw = %f", quat.Qw)
	case quat.Qx != 0:
		t.Errorf("Failed mat1 with qx = %f", quat.Qx)
	case quat.Qy != 0:
		t.Errorf("Failed mat1 with qy = %f", quat.Qy)
	case quat.Qz != 0:
		t.Errorf("Failed mat1 with qz = %f", quat.Qz)
	}

	// Test (accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1)) &&
	// (accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2))
	accumulatortMat2 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 0,
	})
	quat = matrixToQuaterian(accumulatortMat2)
	switch {
	case quat.Qw != 0:
		t.Errorf("Failed mat2 with qw = %f", quat.Qw)
	case quat.Qx != 1:
		t.Errorf("Failed mat2 with qx = %f", quat.Qx)
	case quat.Qy != 0:
		t.Errorf("Failed mat2 with qy = %f", quat.Qy)
	case quat.Qz != 0:
		t.Errorf("Failed mat2 with qz = %f", quat.Qz)
	}

	// Test accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2)
	accumulatortMat3 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, -2, 0,
		0, 0, 0, 1,
	})
	quat = matrixToQuaterian(accumulatortMat3)
	switch {
	case quat.Qw != 0:
		t.Errorf("Failed mat3 with qw = %f", quat.Qw)
	case quat.Qx != 0:
		t.Errorf("Failed mat3 with qx = %f", quat.Qx)
	case quat.Qy != 1:
		t.Errorf("Failed mat3 with qy = %f", quat.Qy)
	case quat.Qz != 0:
		t.Errorf("Failed mat3 with qz = %f", quat.Qz)
	}

	// Test default
	accumulatortMat4 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, -2, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1,
	})
	quat = matrixToQuaterian(accumulatortMat4)
	switch {
	case quat.Qw != 0:
		t.Errorf("Failed mat4 with qw = %f", quat.Qw)
	case quat.Qx != 0:
		t.Errorf("Failed mat4 with qx = %f", quat.Qx)
	case quat.Qy != 0:
		t.Errorf("Failed mat4 with qy = %f", quat.Qy)
	case quat.Qz != 1:
		t.Errorf("Failed mat4 with qz = %f", quat.Qz)
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
