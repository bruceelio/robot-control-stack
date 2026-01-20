Prologue

It’s a disaster!

Our entire chemical supply has been mixed up, and our research projects have ground to a halt. Your mission: Collect and catalogue the samples. But beware, these chemicals are strong – get acidic and basic samples too close together, and they'll react and neutralise each other. But act fast, there are competitors nearby.
Scoring Rules

    The objective of the game, called The Neutral Zone, is to retrieve samples and bring them to laboratories; but to selectively only bring one type of sample.
    The game is played between up to four robots.
    Each match lasts for 150 seconds.
    Robots will be started at the direction of match officials.
    The game is played in the arena specified in the Arena Specifications.
    There are two types of samples in the arena: acidic and basic. There are 8 of each.
    Four of each type of sample are placed on the floor of the arena, and four on the elevated central area.
    Each robot starts fully in its laboratory, in any orientation, and touching at least one arena wall.
    The pH of a laboratory starts at 7 (neutral).
        Each acidic sample in the laboratory reduces the pH by 1.
        Each basic sample in the laboratory increases the pH by 1.
        There is no limit to how high or low the pH of a laboratory can be.
    At the end of the match, robots are scored as follows:
        Robots earn one point for each pH level away from neutral (7) that their laboratory is.
        Robots earn a bonus point for having any sample at all in their laboratory.
        Robots earn a bonus point for at any point moving entirely out of their laboratory.
    A sample is in a laboratory if the vertical projection of the sample overlaps the laboratory, where the tape denoting the laboratory is also included in this area.
    Falling samples are scored where they come to rest.
    At the end of the match, the robot with the most points wins.
    Student Robotics reserves the right to have match officials in the arena during matches.
    The judge’s decision is final.
    Teams may not interact with their robot after the start of a match. This includes to restart their robot should it fail to start. Doing so may result in disqualification from the match.
    While accidental bumps and scrapes are inevitable, the sport is non-contact.
    Robots must not deliberately or negligently damage the arena or anything in it.
    A robot will be stopped during a match when it is at risk of severely damaging itself, other robots, the arena, or otherwise poses a safety risk.

Specifications
Arena

Diagram (See SR_COMP_2026_ARENA.svg)

The arena is a square, shown to scale in the diagram below. The length of the outer walls are 4575±100 mm.
A diagram of the arena, showing locations of key features and graphically indicating the dimensions which are in these specifications.

    All measurements on the diagram are in millimetres. Measurements of the location of items in the arena are relative to its centre.
    The four rectangles in the corners of the arena denote the laboratories. Laboratories are numbered starting from the top left and increasing clockwise.
    Each robot will be assigned a laboratory at the start of every match. Robots may start anywhere fully inside this area, facing any direction, as long as they are physically touching at least one arena wall.
    The perimeter of the arena floor is delimited by the arena wall, which has a minimum height of 220 mm.
    All lines are marked with 48 mm tape using the colours shown in the diagram.
    The floor of the arena is covered with textured, interlocking foam tiles. These will be Nicoman 60 cm EVA Foam Floor Mats 5060608814336. One side is textured, one side is smooth. The tiles will be laid textured-size up. Note that this is a change from previous years when carpet was used.
    Laboratories are 2000±50 mm × 1000±50 mm rectangles in the corners of the arena, with the longer edges falling anticlockwise around the arena.

Markers

Along the arena walls, and on all faces of each sample, are fiducial markers that can be detected with the provided computer vision system. The identifying numbers and sizes of each of these markers are detailed in the table below.
Item	Marker Numbers	Marker Size (mm)
Arena boundary	0 - 19	150
Acidic samples	100-139	80
Basic samples	140-179	80

The markers can be printed on a black-and-white printer.

Each of the arena walls have 5 markers positioned along them as detailed below. These are evenly spaced at a spacing of 762.5±20 mm between marker centres. All arena boundary markers are positioned with the grey border 50±10 mm above the floor.
Central Area

The central area is a raised deck in the middle of the arena, measuring 1220±50 mm × 1220±50 mm. It is elevated 180±30 mm above the floor of the arena. Its walls are solid and have no markers on them.
Samples

    Samples are "single wall" 130±10 mm cardboard box cubes.
    Samples have an 80 mm fiducial marker in the centre of each face.
    The identifier of this marker is the same for all faces.
    Along all edges of the samples is coloured tape to differentiate acidic from basic. Acidic samples are coloured red, and basic samples are coloured blue. This covers the remaining area of the faces that are outside the marker.
    There are 16 samples in total: 8 acidic and 8 basic. 4 of each type are placed on the flood of the arena, and 4 of each type are placed on the central area.
    The samples on the central area are placed with their centres 500 ±50 mm apart along a square pattern, alternating between acids and bases. Acids are placed on the corners of the square pattern, and bases on the edges.
    The samples on the floor are placed with their centres 1000 ±50 mm apart along a square pattern, alternating between acids and bases. Bases are placed on the corners of the square pattern, and acids on the edges.
    Samples on the edges of their respective square patterns are axis-aligned with the arena. Samples on the corners of their respective square patterns are rotated 45° relative to the arena alignment.
    Acid samples are denoted on the diagram, and physically marked in the arena, in red. Basic samples are denoted on the diagram, and physically marked in the arena, in blue. This is based on the colours used in universal indicators.
    Robots may differentiate acidic and basic samples by their fiducial markers. While they also differ by the colour of the tape, this is for human spectators and robots should not rely on colour detection. The tape may be additionally marked with hatching or other patterns to assist colour-blind spectators.
    The arrangement of samples may be seen on the diagram in the arena specification.
    The identifier of each sample in the arena is randomly chosen from the available values in the marker table and where particular samples start may vary between matches.
    Each sample has a unique value within the arena.