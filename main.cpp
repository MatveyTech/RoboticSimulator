#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[])
{
	igl::readOBJ("C:/Users/matvey/Documents/CS2/Graphics/rep3/Data/bunny.obj", V, F);
	//igl::readWRL("C:/Users/matvey/Documents/CS2/Graphics/rep3/Data/bunny.obj", V, F);

	// Plot the mesh
	igl::opengl::glfw::Viewer viewer;
	viewer.data().set_mesh(V, F);
	const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 3) <<
		2.0, 2.0, 2.0,
		5.0, 5.0, 5.0).finished();
	viewer.data().add_points(P, Eigen::RowVector3d(255, 0, 0));
	viewer.launch();
	return 0;
}
