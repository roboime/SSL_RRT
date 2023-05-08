#include "Node.hpp"
#include <limits>

int _max_nodes_near = 20;

flann::Index<flann::L2<double>> _kdtree(flann::KDTreeIndexParams(4));
unordered_map<vector<double>, Node*, hash_vector<double>> _nodemap;

vector<Node*> find_nodes_near(Node node, double radius, int buffer_size = 5){
    radius *= radius;
    int nn = buffer_size;
    vector<Node*> ret;
    for(auto p : node._vec){
        if(isinf(p) || isnan(p)){
            cerr << "inf ou nan\n";
            return ret;
        }
    }
    flann::Matrix<double> query(node._vec.data(), 1, 2);
    vector<int> ind(query.rows*nn, -1);
    flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
    flann::Matrix<double> dists(new double[query.rows*nn], query.rows, nn);
    // do a knn search, using 128 checks
    //_kdtree.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
    _kdtree.radiusSearch(query, indices, dists, radius, flann::SearchParams(128));

    for(int i=0;i<nn;i++){
        double* point;
        if(indices[0][i] != -1) point = _kdtree.getPoint(indices[0][i]);
        else break;

        if(1){
            vector<double> temp(point, point+2);
            ret.push_back(_nodemap[temp]);
        }

    }
    delete[] indices.ptr();
    delete[] dists.ptr();
  
    return ret;
}

int main() {
    vector<double> a = {500, 500};
    vector<double> b = {0, 0};
    vector<double> c = {100, 100};
    vector<double> d = {-100, -100};
    vector<double> e = {-100, 100};
    vector<double> f = {100, -100};
    vector<double> g = {500, -500};
    vector<double> h = {-500, -500};
    Node a_vec(a[0], a[1], nullptr);
    Node b_vec(b[0], b[1], nullptr);
    Node c_vec(c[0], c[1], nullptr);
    Node d_vec(d[0], d[1], nullptr);
    Node e_vec(e[0], e[1], nullptr);
    Node f_vec(f[0], f[1], nullptr);
    Node g_vec(g[0], g[1], nullptr);
    Node h_vec(h[0], h[1], nullptr);

    _nodemap[a] = &a_vec;
    _nodemap[b] = &b_vec;
    _nodemap[c] = &c_vec;
    _nodemap[d] = &d_vec;
    _nodemap[e] = &e_vec;
    _nodemap[f] = &f_vec;
    _nodemap[g] = &g_vec;
    _nodemap[h] = &h_vec;

    _kdtree.buildIndex(flann::Matrix<double>(e.data(), 1, 2));
    _kdtree.addPoints(flann::Matrix<double>(c.data(), 1, 2));
    _kdtree.addPoints(flann::Matrix<double>(b.data(), 1, 2));
    _kdtree.addPoints(flann::Matrix<double>(d.data(), 1, 2));
    _kdtree.addPoints(flann::Matrix<double>(a.data(), 1, 2));
    _kdtree.addPoints(flann::Matrix<double>(f.data(), 1, 2));
    _kdtree.addPoints(flann::Matrix<double>(g.data(), 1, 2));
    _kdtree.addPoints(flann::Matrix<double>(h.data(), 1, 2));
    //flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
    //flann::Matrix<double> dataset(arr, 8, 2);

    //Node nd(numeric_limits<double>::quiet_NaN(), numeric_limits<double>::quiet_NaN(), nullptr);
    Node nd(5,5,nullptr);
    vector<double> vec = {25,25};
    //find_nodes_near(nd, 1000);
    
    for(auto p : find_nodes_near(nd, 1000)){
        cout << p->_x << " " << p->_y << "\n";
    }
}