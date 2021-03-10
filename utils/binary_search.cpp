//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2021 MUR Driverless. All rights reserved.
//
#include "binary_search.h"

int mpcc::binary_search_left(const VectorXd &arr, double x) {
    int lo = 0;
    int hi = arr.size();
    int mid;

    if (x < arr(lo)) {
        std::cout << "Binary search for x = " << x << " is outside minimum range" << std::endl;
        x = arr(lo);
    }
    else if (x > arr(hi-1)) {
        std::cout << "Binary search for x = " << x << " is outside maximum range" << std::endl;
        x = arr(hi-1);
    }

    // Taken from Python's bisect.bisect_left()
    while (lo < hi) {
        mid = (int) (lo + hi) / 2;
        if (arr(mid) < x) { lo = mid + 1; }
        else { hi = mid; }
    }

    // For some reason, bisect.bisect_left() returns the right element
    // So we decide to return this index - 1
    if (lo > 0) {
        return lo - 1;
    }
    // But, if the index is 0, we return it as it is
    else {
        return lo;
    }
}
