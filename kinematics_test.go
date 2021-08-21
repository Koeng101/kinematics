package kinematics

import (
	"fmt"
	"gonum.org/v1/gonum/mat"
	"math/rand"
	"testing"
)

func TestForwardKinematics(t *testing.T) {
	testThetas := []float64{10, 1, 1, 0, 0, 0}
	f := ForwardKinematics(testThetas, AR3DhParameters)
	switch {
	case f.X != -101.74590611879692:
		t.Errorf("Forward kinematics failed on f.X = %f", f.X)
	case f.Y != -65.96805988175777:
		t.Errorf("Forward kinematics failed on f.Y = %f", f.Y)
	case f.Z != -322.27756822304093:
		t.Errorf("Forward kinematics failed on f.Z = %f", f.Z)
	case f.Qx != 0.06040824945687102:
		t.Errorf("Forward kinematics failed on f.Qx = %f", f.Qx)
	case f.Qy != -0.20421099379003957:
		t.Errorf("Forward kinematics failed on f.Qy = %f", f.Qy)
	case f.Qz != 0.2771553334491873:
		t.Errorf("Forward kinematics failed on f.Qz = %f", f.Qz)
	case f.Qw != 0.9369277637862541:
		t.Errorf("Forward kinematics failed on f.Qw = %f", f.Qw)
	}
}

func TestInverseKinematics(t *testing.T) {
	desiredEndEffector := XyzWxyz{-91.72345062922584, 386.93155027870745, 382.30917872225154, 0.4007833787652043, -0.021233218878182854, 0.9086418268616911, 0.41903052745255764}
	_, err := InverseKinematics(desiredEndEffector, AR3DhParameters)
	if err != nil {
		t.Errorf("Inverse Kinematics failed with error: %s", err)
	}

	// This case should fail because the X required is too large
	desiredEndEffector = XyzWxyz{-91000000.72345062922584, 386.93155027870745, 382.30917872225154, 0.4007833787652043, -0.021233218878182854, 0.9086418268616911, 0.41903052745255764}
	_, err = InverseKinematics(desiredEndEffector, AR3DhParameters)
	if err == nil {
		t.Errorf("Inverse Kinematics should have failed with large X")
	}
}

func TestMatrixToQuaterian(t *testing.T) {
	var qw float64
	var qx float64
	var qy float64
	var qz float64

	// Test tr > 0
	accumulatortMat1 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat1)
	switch {
	case qw != 1:
		t.Errorf("Failed mat1 with qw = %f", qw)
	case qx != 0:
		t.Errorf("Failed mat1 with qx = %f", qx)
	case qy != 0:
		t.Errorf("Failed mat1 with qy = %f", qy)
	case qz != 0:
		t.Errorf("Failed mat1 with qz = %f", qz)
	}

	// Test (accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1)) && (accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2))
	accumulatortMat2 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 0,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat2)
	switch {
	case qw != 0:
		t.Errorf("Failed mat2 with qw = %f", qw)
	case qx != 1:
		t.Errorf("Failed mat2 with qx = %f", qx)
	case qy != 0:
		t.Errorf("Failed mat2 with qy = %f", qy)
	case qz != 0:
		t.Errorf("Failed mat2 with qz = %f", qz)
	}

	// Test accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2)
	accumulatortMat3 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, -2, 0,
		0, 0, 0, 1,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat3)
	switch {
	case qw != 0:
		t.Errorf("Failed mat3 with qw = %f", qw)
	case qx != 0:
		t.Errorf("Failed mat3 with qx = %f", qx)
	case qy != 1:
		t.Errorf("Failed mat3 with qy = %f", qy)
	case qz != 0:
		t.Errorf("Failed mat3 with qz = %f", qz)
	}

	// Test default
	accumulatortMat4 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, -2, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat4)
	switch {
	case qw != 0:
		t.Errorf("Failed mat4 with qw = %f", qw)
	case qx != 0:
		t.Errorf("Failed mat4 with qx = %f", qx)
	case qy != 0:
		t.Errorf("Failed mat4 with qy = %f", qy)
	case qz != 1:
		t.Errorf("Failed mat4 with qz = %f", qz)
	}
}

func BenchmarkInverseKinematics(b *testing.B) {
	randTheta := func(joint int) float64 {
		var randomTheta float64
		for r := float64(-1); !(r < AR3DhParameters.StepperLimits[joint]*AR3DhParameters.StepsPerAngleDegree[joint]) || r < 0; {
			r = 360 * rand.Float64()
			r = randomTheta
		}
		return randomTheta
	}
	for i := 0; i < b.N; i++ {
		randomSeed := []float64{randTheta(0), randTheta(1), randTheta(2), randTheta(3), randTheta(4), randTheta(5)}
		desiredEndEffector := ForwardKinematics(randomSeed, AR3DhParameters)
		_, err := InverseKinematics(desiredEndEffector, AR3DhParameters)
		if err != nil {
			b.Errorf("Failed inverse kinematics benchmark with: %s\nSeed: %f-%f-%f-%f-%f-%f", err, randomSeed[0], randomSeed[1], randomSeed[2], randomSeed[3], randomSeed[4], randomSeed[5])
		}
	}
}

func ExampleForwardKinematics() {
	angles := []float64{10, 1, 1, 0, 0, 0}
	coordinates := ForwardKinematics(angles, AR3DhParameters)

	fmt.Println(coordinates)
	// Output: {-101.74590611879692 -65.96805988175777 -322.27756822304093 0.06040824945687102 -0.20421099379003957 0.2771553334491873 0.9369277637862541}
}

func ExampleInverseKinematics() {
	coordinates := XyzWxyz{X: -100, Y: 250, Z: 250, Qx: 0.4007833787652043, Qy: -0.021233218878182854, Qz: 0.9086418268616911, Qw: 0.41903052745255764}
	angles, _ := InverseKinematics(coordinates, AR3DhParameters)

	fmt.Println(angles)
	// Output: [1.8462740950010432 0.3416721655970939 -2.313720459511564 -1.7765008677785283 2.2218097507147707 1.2318789996199948]
}
