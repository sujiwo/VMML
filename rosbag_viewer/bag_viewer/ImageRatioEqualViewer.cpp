/*
 * Copyright 2019 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include "ImageRatioEqualViewer.h"


ImageRatioEqualViewer::ImageRatioEqualViewer(QWidget *parent):
    QLabel(parent)
{
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
}


void ImageRatioEqualViewer::drawImage()
{
    auto rect = contentsRect();
    double imageAspectRatio = (double)frImageCopy.width() / (double)frImageCopy.height();
    
    double zfactor;
    if (rect.width() > rect.height()) {
        zfactor = (double)rect.height() / (double)frImageCopy.height();
    } else {
        zfactor = (double)rect.width() / (double)frImageCopy.width();
    }
    
    int 
        newWidth = zfactor * frImageCopy.width(),
        newHeight = zfactor * frImageCopy.height();
    
    QImage currentImageResized = frImageCopy.scaled(newWidth, newHeight);
    
    this->setPixmap(QPixmap::fromImage(currentImageResized));
}



void ImageRatioEqualViewer::setImage(const QImage& image)
{
    frImageCopy = image.copy();
    drawImage();
}


void ImageRatioEqualViewer::paintEvent(QPaintEvent* evt)
{
    drawImage();
    QLabel::paintEvent(evt);
}

