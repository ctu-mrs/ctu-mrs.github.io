---
layout: default
title: Good practices in C++
parent: Introduction
nav_order: 1
---

# Good practices in modern C++ for the purposes of MRS

This guide will attempt to summarize good practices to use and bad practices to avoid when writing C++ code in the context of ROS, robotics and research.
The guide is mainly targeted at people who are coming to C++ from C or who are still using old-style C++.

A list of the main tackled topics is:

 * [Avoid raw pointers, `new`, `malloc`, etc. like the devil.](#raw-pointers)
 * [Take function parameters by constant referece (or constant copy in case of primitive types).](#function-parameters)
 * [Thread synchronization](#thread-synchronization)
 * TODO: ROS-related stuff
 * [Other tips and remarks.](#other-minor-remarks)

## Dynamic memory management

TODO: rewrite and finish

To avoid unnecessary copies when passing data to a function in C, it's common to use pointers.
C++ provides references, which are generally safer than pointers, since it's harder to accidentally create an invalid reference than a pointer.
Using references makes the data ownership and lifespan of variables clear and less error-prone.

## Function parameters

The rules of thumb when defining function parameters is:

  1. If you're taking a primitive type as a parameter (e.g. `int`, `float`, `bool` etc.), use a constant copy:
     ```
     bool foo(const int a, const float b);
     ```
  2. If you're taking a class/struct, use a constant reference:
     ```
     class Bar, Baz;
     Bar foo(const int a, const Baz& b);
     ```
  3. If you need to return multiple variables, there are several possibilities:
     ```
     std::tuple<bool, float> foo2(const int a);
     
     (...)
     
     // preffered way since it's clearer what is input and what output
     // and all can be const, avoiding accidental modification
     const int a = 5;
     const auto [c, b] = foo1(a);
     ```
     or
     ```
     bool foo1(const int a, float& ret_b);
     
     (...)
     
     // less elegant and less clear way, but valid
     const int a = 5;
     float b;
     const bool c = foo1(a, b);
     ```
  4. If you want to modify a passed parameter by a function (e.g. use the function to update an object's value), use a reference, but make this clear:
     ```
     void update_values(std::vector<float>& a, const float update_val)
     {
       for (const auto& el : a)
         el *= update_val;
     }
     ```
## Thread synchronization

If you have a single primitive-type variable which you want to access and modify from multiple threads in a thread-safe manner (e.g. some counter or a flag that a thread is running), use `std::atomic<T>`.

For thread synchronization where you want the thread to become inactive while waiting for another thread to release a resource, use `std::mutex` and `std::lock_guard`.

TODO: `std::condition_variable`

Do not use `volatile` (unless working with a microcontroller, where you really need it).
Note that `volatile` does *NOT* ensure any kind of thread safety!

TODO: explain why

## Other minor remarks

 * Turn on `-Wall` and make your code warning-less.
   The warnings are there to tell you about potential code smell, so do not ignore them.
 * Use `const` whenever possible.
   This way you will avoid accidentally modifying variables which are not supposed to be modified and enable the compiler to better optimize.
