#!/bin/bash

CMD="astyle --options=.astylerc"

$CMD rosplane/src/*
$CMD rosplane/include/*
$CMD rosplane_sim/src/*
$CMD rosplane_sim/include/rosplane_sim/*
