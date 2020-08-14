/*
Copyright (c) 2011 Juan Mellado

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

/*
References:
- "ArUco: a minimal library for Augmented Reality applications based on OpenCv"
  http://www.uco.es/investiga/grupos/ava/node/26
*/

var CV = require('./cv');

var AR = AR || {};

AR.Marker = function(id, corners){
  this.id = id;
  this.corners = corners;
};

AR.Detector = function(){
  this.grey = new CV.Image();
  this.thres = new CV.Image();
  this.homography = new CV.Image();
  this.binary = [];
  this.contours = [];
  this.polys = [];
  this.candidates = [];
};

AR.Detector.prototype.detect = function(image, markerType){
  markerType = typeof markerType === 'undefined' ? 5 : markerType;  // markerType values : 3 (for 3x3), 4 (for 4x4) or 5 (for 5x5, by default)
  CV.grayscale(image, this.grey);
  CV.adaptiveThreshold(this.grey, this.thres, 1+Math.floor(image.width/800), 7);

  this.contours = CV.findContours(this.thres, this.binary);

  this.candidates = this.keepBigHoles(this.contours, (markerType+2)*4);

  this.candidates = this.findCandidates(this.candidates, image.width * 0.16, 0.05, (markerType+2));
  this.candidates = this.clockwiseCorners(this.candidates);
  this.candidates = this.notTooNear(this.candidates, image.width * 0.02);

  return this.findMarkers(this.grey, this.candidates, 49, markerType);
};

AR.Detector.prototype.keepBigHoles = function(contours, minSize){
    var candidates = [], len = contours.length, contour, poly, i;

    for (i = 0; i < len; ++ i){
        contour = contours[i];
//        if (contour.length >= minSize && contour.hole) {
        if (contour.length >= minSize) {
            candidates.push(contour);
        }
    }
    return candidates;
};

AR.Detector.prototype.findCandidates = function(contours, minSize, epsilon, minLength){
  var candidates = [], len = contours.length, contour, poly, i;

  this.polys = [];

  for (i = 0; i < len; ++ i){
    contour = contours[i];

    if (contour.length >= minSize){
      poly = CV.approxPolyDP(contour, contour.length * epsilon);

      var [square, isSquare] = this.getSquare(poly);

      this.polys.push(poly);
//      this.polys.push(square);

//      if ( (4 === poly.length) && ( CV.isContourConvex(poly) ) ){
      if ( isSquare ){
//      if ( poly.length < 10 ){
//        if ( CV.minEdgeLength(poly) >= minLength){
//          candidates.push(poly);
        if ( CV.minEdgeLength(square) >= minLength){
          candidates.push(square);
        }
      }
    }
  }

  return candidates;
};

AR.Detector.prototype.getSquare = function(contour) {
    var xmax = 0, xmin = 999999, ymax = 0, ymin = 999999;
    var gmax = 0, gmin = 999999, gmax2 = -99999999, gmin2 = 99999999;
    var g = 0, g2 = 0;
    var right, left, top, bot;
    var topRight, topLeft, botRight, botLeft;
    var gBotLeft, gTopRight, gTopLeft, gBotRight;


    for(i=0; i<contour.length; i++) {
        let pt = contour[i];
        if (pt.x > xmax) {
            xmax = pt.x;
            right = pt;
        }
        if (pt.y > ymax) {
            ymax = pt.y;
            bot = pt;
        }
        if (pt.x < xmin) {
            xmin = pt.x;
            left = pt;
        }
        if (pt.y < ymin) {
            ymin = pt.y;
            top = pt;
        }

        g = pt.x+pt.y;
        if (g < gmin) {
            gmin = g;
            gTopLeft = pt;
        }
        if (g > gmax) {
            gmax = g;
            gBotRight = pt;
        }
        g2 = pt.x-pt.y;
        if (g2 < gmin2) {
            gmin2 = g2;
            gBotLeft = pt;
        }
        if (g2 > gmax2) {
            gmax2 = g2;
            gTopRight = pt;
        }
    }

    if (top.x > bot.x) {
        topRight = top;
        topLeft = left;
        botRight = right;
        botLeft = bot;
    } else {
        topRight = right;
        topLeft = top;
        botRight = bot;
        botLeft = left;
    }

    var width = Math.sqrt( Math.pow(topLeft.x - topRight.x, 2) + Math.pow(topLeft.y - topRight.y, 2));
    var height = Math.sqrt( Math.pow(botRight.x - topRight.x, 2) + Math.pow(botRight.y - topRight.y, 2));
    var sizeDiscrepancy = width+height / 3;
    var isSquare = (Math.abs(width-height)<sizeDiscrepancy);

    return ([[ gTopRight, gBotRight, gBotLeft, gTopLeft ], isSquare ]);
};

AR.Detector.prototype.clockwiseCorners = function(candidates){
  var len = candidates.length, dx1, dx2, dy1, dy2, swap, i;

  for (i = 0; i < len; ++ i){
    dx1 = candidates[i][1].x - candidates[i][0].x;
    dy1 = candidates[i][1].y - candidates[i][0].y;
    dx2 = candidates[i][2].x - candidates[i][0].x;
    dy2 = candidates[i][2].y - candidates[i][0].y;

    if ( (dx1 * dy2 - dy1 * dx2) < 0){
      swap = candidates[i][1];
      candidates[i][1] = candidates[i][3];
      candidates[i][3] = swap;
    }
  }

  return candidates;
};

AR.Detector.prototype.notTooNear = function(candidates, minDist){
  var notTooNear = [], len = candidates.length, dist, dx, dy, i, j, k;

  for (i = 0; i < len; ++ i){

    for (j = i + 1; j < len; ++ j){
      dist = 0;

      for (k = 0; k < 4; ++ k){
        dx = candidates[i][k].x - candidates[j][k].x;
        dy = candidates[i][k].y - candidates[j][k].y;

        dist += dx * dx + dy * dy;
      }

      if ( (dist / 4) < (minDist * minDist) ){

        if ( CV.perimeter( candidates[i] ) < CV.perimeter( candidates[j] ) ){
          candidates[i].tooNear = true;
        }else{
          candidates[j].tooNear = true;
        }
      }
    }
  }

  for (i = 0; i < len; ++ i){
    if ( !candidates[i].tooNear ){
      notTooNear.push( candidates[i] );
    }
  }

  return notTooNear;
};

AR.Detector.prototype.findMarkers = function(imageSrc, candidates, warpSize, markerType){
  var markers = [], len = candidates.length, candidate, marker, i;

  for (i = 0; i < len; ++ i){
    candidate = candidates[i];

    CV.warp(imageSrc, this.homography, candidate, warpSize);

    CV.threshold(this.homography, this.homography, CV.otsu(this.homography) );

    marker = this.getMarker(this.homography, candidate, markerType);
    if (marker){
      markers.push(marker);
    }
  }

  return markers;
};

AR.Detector.prototype.getMarker = function(imageSrc, candidate, markerType){
  var width = (imageSrc.width / (markerType+2)) >>> 0,
      minZero = (width * width) >> 1,
      bits = [], rotations = [], distances = [],
      square, pair, inc, i, j;

  for (i = 0; i < markerType+2; ++ i){
    inc = (0 === i || markerType+1 === i)? 1: markerType+1;

    for (j = 0; j < markerType+2; j += inc){
      square = {x: j * width, y: i * width, width: width, height: width};
      if ( CV.countNonZero(imageSrc, square) > minZero){
        return null;
      }
    }
  }

  for (i = 0; i < markerType; ++ i){
    bits[i] = [];

    for (j = 0; j < markerType; ++ j){
      square = {x: (j + 1) * width, y: (i + 1) * width, width: width, height: width};

      bits[i][j] = CV.countNonZero(imageSrc, square) > minZero? 1: 0;
    }
  }

  rotations[0] = bits;
  distances[0] = this.hammingDistance( rotations[0]);

  pair = {first: distances[0], second: 0};

  for (i = 1; i < 4; ++ i){
    rotations[i] = this.rotate( rotations[i - 1] );
    distances[i] = this.hammingDistance( rotations[i]);

    if (distances[i] < pair.first){
      pair.first = distances[i];
      pair.second = i;
    }
  }

  if (0 !== pair.first){
    return null;
  }

  return new AR.Marker(
    this.mat2id( rotations[pair.second] ),
    this.rotate2(candidate, 4 - pair.second) );
};

AR.Detector.prototype.hammingDistance = function(bits){
  var idsList = {
        5: [ [1,0,0,0,0], [1,0,1,1,1], [0,1,0,0,1], [0,1,1,1,0] ],
        4: [ [0,0,1,0], [0,0,1,1], [1,1,1,0], [1,0,1,1]],
        3: [[0,1,0], [0,0,1], [1,1,0], [1,1,1]]
      },
      len = bits.length,
      dist = 0, sum, minSum, i, j, k;

  for (i = 0; i < len; ++ i){
    minSum = Infinity;

    for (j = 0; j < 4; ++ j){
      sum = 0;

      for (k = 0; k < len; ++ k){
          sum += bits[i][k] === idsList[len][j][k]? 0: 1;
      }

      if (sum < minSum){
        minSum = sum;
      }
    }

    dist += minSum;
  }

  return dist;
};

AR.Detector.prototype.mat2id = function(bits){
  var id = 0, i;
  var bitsToRead = {
        5: [1,3],
        4: [0,3],
        3: [0,2]
      },
      len = bits.length;

  for (i = 0; i < len; ++ i){
    id <<= 1;
    id |= bits[i][bitsToRead[len][0]];
    id <<= 1;
    id |= bits[i][bitsToRead[len][1]];
  }

  return id;
};

AR.Detector.prototype.rotate = function(src){
  var dst = [], len = src.length, i, j;

  for (i = 0; i < len; ++ i){
    dst[i] = [];
    for (j = 0; j < src[i].length; ++ j){
      dst[i][j] = src[src[i].length - j - 1][i];
    }
  }

  return dst;
};

AR.Detector.prototype.rotate2 = function(src, rotation){
  var dst = [], len = src.length, i;

  for (i = 0; i < len; ++ i){
    dst[i] = src[ (rotation + i) % len ];
  }

  return dst;
};

module.exports = AR;