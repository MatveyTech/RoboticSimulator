#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/opengl/gl.h>
//#include <igl/opengl2/draw_mesh.h>

using namespace Eigen;
using namespace std;




void TransformPoints(Eigen::MatrixXd &V, Eigen::Matrix4d &tr)
{
	for (int i = 0; i < V.count() / 3; ++i)
	{
		Vector4d vec(V(i, 0), V(i, 1), V(i, 2), 1);
		Vector4d transformed = tr*vec;
		V(i, 0) = transformed(0);
		V(i, 1) = transformed(1);
		V(i, 2) = transformed(2);
	}
}

int main(int argc, char *argv[])
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	igl::readOBJ("C:/Users/matvey/Documents/CS2/Graphics/rep3/Data/bunny.obj", V, F);

	igl::opengl::glfw::Viewer viewer;
	Matrix4d tr;
	tr << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	viewer.callback_key_down =
		[&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
	{
		if (key == GLFW_KEY_ENTER)
		{
			tr(0, 3) = tr(0, 3) + 0.15;
			TransformPoints(V, tr);
			viewer.data_list[0].set_mesh(V, F);
			return true;
		}
		return false;
	};
	
	
	viewer.data_list[0].set_mesh(V, F);
	viewer.data_list[0].set_colors(Eigen::RowVector3d(1, 0, 0));
	//viewer.append_mesh();

	const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 3) <<
		2.0, 2.0, 2.0,
		5.0, 5.0, 5.0).finished();
	//viewer.data().add_points(P, Eigen::RowVector3d(255, 0, 0));
	viewer.launch();
	return 0;
}
