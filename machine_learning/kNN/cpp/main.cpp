#include "kdtree.h"
#include <stdio.h>
#include <stdlib.h>

// reference to https://www.programmersought.com/article/1629202664/
 
int main() {
    float datas[100] = {1.3, 1.3, 1.3,
                         8.3, 8.3, 8.3,
                         2.3, 2.3, 2.3,
                         1.2, 1.2, 1.2,
                         7.3, 7.3, 7.3,
                         9.3, 9.3, 9.3,
                         15, 15, 15,
                         3, 3, 3,
                         1.1, 1.1, 1.1,
                         12, 12, 12,
                         4, 4, 4,
                         5, 5, 5};
    float labels[100];
    for(size_t i = 0; i < 12; ++i)
        labels[i] = (float)i;
    tree_model *model = build_kdtree(datas, labels, 12, 3, 2);
    float test[6] = {3, 3, 3, 4, 4, 4};
    size_t args[100];
    float dists[100];
         Find_k_nearests(model, test, 5, args, dists); // This only searches for K neighbors of (3,3,3)
 
    printf("K-Nearest: \n");
    for (size_t i = 0; i < 5; ++i) {
        printf("ID %d, Dist %.2f\n", args[i], dists[i]);
    }
 
         Float *ans = k_nearests_neighbor(model, test, 2, 5, false); // Form 2 indicates: 2 samples in test are to be tested
    printf("k Nearest Neighbor Regressor: \n%.2f %.2f\n", ans[0], ans[1]);
 
//    tree_node *root = model->root;
 
 
    free(ans);
    free_tree_memory(model->root);
 
    return 0;
}
