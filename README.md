[vtk图像处理](http://tanqingbo.com/2017/08/24/vtkPolyData%E6%95%B0%E6%8D%AE%E5%A4%84%E7%90%86/) 

[svd空间离散点合成平面集合](https://blog.csdn.net/yanxiaopan/article/details/68489560) 

[最小二乘法拟合平面](https://blog.csdn.net/zhouyelihua/article/details/46122977) 


```csharp
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Controls.Primitives;
using System.Windows.Media;

namespace VirtualizingPPanel
{

    public class VirtualizingWrapPanel : VirtualizingPanel, IScrollInfo
    {
        private TranslateTransform trans = new TranslateTransform();

        public VirtualizingWrapPanel()
        {
            this.RenderTransform = trans;
        }

        #region DependencyProperties
        public static readonly DependencyProperty ChildWidthProperty = DependencyProperty.RegisterAttached("ChildWidth", typeof(double), typeof(VirtualizingWrapPanel), new FrameworkPropertyMetadata(200.0, FrameworkPropertyMetadataOptions.AffectsMeasure | FrameworkPropertyMetadataOptions.AffectsArrange));

        public static readonly DependencyProperty ChildHeightProperty = DependencyProperty.RegisterAttached("ChildHeight", typeof(double), typeof(VirtualizingWrapPanel), new FrameworkPropertyMetadata(200.0, FrameworkPropertyMetadataOptions.AffectsMeasure | FrameworkPropertyMetadataOptions.AffectsArrange));

        //鼠标每一次滚动 UI上的偏移
        public static readonly DependencyProperty ScrollOffsetProperty = DependencyProperty.RegisterAttached("ScrollOffset", typeof(int), typeof(VirtualizingWrapPanel), new PropertyMetadata(10));

        public int ScrollOffset
        {
            get { return Convert.ToInt32(GetValue(ScrollOffsetProperty)); }
            set { SetValue(ScrollOffsetProperty, value); }
        }
        public double ChildWidth
        {
            get => Convert.ToDouble(GetValue(ChildWidthProperty));
            set => SetValue(ChildWidthProperty, value);
        }
        public double ChildHeight
        {
            get => Convert.ToDouble(GetValue(ChildHeightProperty));
            set => SetValue(ChildHeightProperty, value);
        }
        #endregion

        int GetItemCount(DependencyObject element)
        {
            var itemsControl = ItemsControl.GetItemsOwner(element);
            return itemsControl.HasItems ? itemsControl.Items.Count : 0;
        }
        int CalculateChildrenPerRow(Size availableSize)
        {
            int childPerRow = 0;
            if (availableSize.Width == double.PositiveInfinity)
                childPerRow = this.Children.Count;
            else
                childPerRow = Math.Max(1, Convert.ToInt32(Math.Floor(availableSize.Width / this.ChildWidth)));
            return childPerRow;
        }
        /// <summary>
        /// width不超过availableSize的情况下，自身实际需要的Size(高度可能会超出availableSize)
        /// </summary>
        /// <param name="availableSize"></param>
        /// <param name="itemsCount"></param>
        /// <returns></returns>
        Size CalculateExtent(Size availableSize, int itemsCount)
        {
            int childPerRow = CalculateChildrenPerRow(availableSize);//现有宽度下 一行可以最多容纳多少个
            return new Size(childPerRow * this.ChildWidth, this.ChildHeight * Math.Ceiling(Convert.ToDouble(itemsCount) / childPerRow));
        }
        /// <summary>
        /// 更新滚动条
        /// </summary>
        /// <param name="availableSize"></param>
        void UpdateScrollInfo(Size availableSize)
        {
            var extent = CalculateExtent(availableSize, GetItemCount(this));//extent 自己实际需要
            if (extent != this.extent)
            {
                this.extent = extent;
                this.ScrollOwner.InvalidateScrollInfo();
            }
            if (availableSize != this.viewPort)
            {
                this.viewPort = availableSize;
                this.ScrollOwner.InvalidateScrollInfo();
            }
        }
        /// <summary>
        /// 获取所有item，在可视区域内第一个item和最后一个item的索引
        /// </summary>
        /// <param name="firstIndex"></param>
        /// <param name="lastIndex"></param>
        void GetVisiableRange(ref int firstIndex, ref int lastIndex)
        {
            int childPerRow = CalculateChildrenPerRow(this.extent);
            firstIndex = Convert.ToInt32(Math.Floor(this.offset.Y / this.ChildHeight)) * childPerRow;
            lastIndex = Convert.ToInt32(Math.Ceiling((this.offset.Y + this.viewPort.Height) / this.ChildHeight)) * childPerRow - 1;
            int itemsCount = GetItemCount(this);
            if (lastIndex >= itemsCount)
                lastIndex = itemsCount - 1;

        }
        /// <summary>
        /// 将不在可视区域内的item 移除
        /// </summary>
        /// <param name="startIndex">可视区域开始索引</param>
        /// <param name="endIndex">可视区域结束索引</param>
        void CleanUpItems(int startIndex, int endIndex)
        {
            var children = this.InternalChildren;
            var generator = this.ItemContainerGenerator;
            for (int i = children.Count - 1; i >= 0; i--)
            {
                var childGeneratorPosi = new GeneratorPosition(i, 0);
                int itemIndex = generator.IndexFromGeneratorPosition(childGeneratorPosi);

                if (itemIndex < startIndex || itemIndex > endIndex)
                {

                    generator.Remove(childGeneratorPosi, 1);
                    RemoveInternalChildRange(i, 1);
                }
            }
        }
        /// <summary>
        /// scroll/availableSize/添加删除元素 改变都会触发  edit元素不会改变
        /// </summary>
        /// <param name="availableSize"></param>
        /// <returns></returns>
        protected override Size MeasureOverride(Size availableSize)
        {
            this.UpdateScrollInfo(availableSize);//availableSize更新后，更新滚动条
            int firstVisiableIndex = 0, lastVisiableIndex = 0;
            GetVisiableRange(ref firstVisiableIndex, ref lastVisiableIndex);//availableSize更新后，获取当前viewport内可放置的item的开始和结束索引  firstIdnex-lastIndex之间的item可能部分在viewport中也可能都不在viewport中。

            UIElementCollection children = this.InternalChildren;//因为配置了虚拟化，所以children的个数一直是viewport区域内的个数,如果没有虚拟化则是ItemSource的整个的个数
            IItemContainerGenerator generator = this.ItemContainerGenerator;
            //获得第一个可被显示的item的位置
            GeneratorPosition startPosi = generator.GeneratorPositionFromIndex(firstVisiableIndex);
            int childIndex = (startPosi.Offset == 0) ? startPosi.Index : startPosi.Index + 1;//startPosi在chilren中的索引
            using (generator.StartAt(startPosi, GeneratorDirection.Forward, true))
            {
                int itemIndex = firstVisiableIndex;
                while (itemIndex <= lastVisiableIndex)//生成lastVisiableIndex-firstVisiableIndex个item
                {
                    bool newlyRealized = false;
                    var child = generator.GenerateNext(out newlyRealized) as UIElement;
                    if (newlyRealized)
                    {
                        if (childIndex >= children.Count)
                            base.AddInternalChild(child);
                        else
                        {
                            base.InsertInternalChild(childIndex, child);
                        }
                        generator.PrepareItemContainer(child);
                    }
                    else
                    {
                        //处理 正在显示的child被移除了这种情况
                        if (!child.Equals(children[childIndex]))
                        {
                            base.RemoveInternalChildRange(childIndex, 1);
                        }
                    }
                    child.Measure(new Size(this.ChildWidth, this.ChildHeight));
                    //child.DesiredSize//child想要的size
                    itemIndex++;
                    childIndex++;
                }
            }
            CleanUpItems(firstVisiableIndex, lastVisiableIndex);
            return new Size(double.IsInfinity(availableSize.Width) ? 0 : availableSize.Width, double.IsInfinity(availableSize.Height) ? 0 : availableSize.Height);//自身想要的size
        }
        protected override Size ArrangeOverride(Size finalSize)
        {
            Debug.WriteLine("----ArrangeOverride");
            var generator = this.ItemContainerGenerator;
            UpdateScrollInfo(finalSize);
            int childPerRow = CalculateChildrenPerRow(finalSize);
            double availableItemWidth = finalSize.Width / childPerRow;
            for (int i = 0; i <= this.Children.Count - 1; i++)
            {
                var child = this.Children[i];
                int itemIndex = generator.IndexFromGeneratorPosition(new GeneratorPosition(i, 0));
                int row = itemIndex / childPerRow;//current row
                int column = itemIndex % childPerRow;
                double xCorrdForItem = 0;

                xCorrdForItem = column * availableItemWidth + (availableItemWidth - this.ChildWidth) / 2;

                Rect rec = new Rect(xCorrdForItem, row * this.ChildHeight, this.ChildWidth, this.ChildHeight);
                child.Arrange(rec);
            }
            return finalSize;
        }
        protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
        {
            base.OnRenderSizeChanged(sizeInfo);
            this.SetVerticalOffset(this.VerticalOffset);
        }
        protected override void OnClearChildren()
        {
            base.OnClearChildren();
            this.SetVerticalOffset(0);
        }
        protected override void BringIndexIntoView(int index)
        {
            if (index < 0 || index >= Children.Count)
                throw new ArgumentOutOfRangeException();
            int row = index / CalculateChildrenPerRow(RenderSize);
            SetVerticalOffset(row * this.ChildHeight);
        }
        #region IScrollInfo Interface
        public bool CanVerticallyScroll { get; set; }
        public bool CanHorizontallyScroll { get; set; }

        private Size extent = new Size(0, 0);
        public double ExtentWidth => this.extent.Width;

        public double ExtentHeight => this.extent.Height;

        private Size viewPort = new Size(0, 0);
        public double ViewportWidth => this.viewPort.Width;

        public double ViewportHeight => this.viewPort.Height;

        private Point offset;
        public double HorizontalOffset => this.offset.X;

        public double VerticalOffset => this.offset.Y;

        public ScrollViewer ScrollOwner { get; set; }

        public void LineDown()
        {
            this.SetVerticalOffset(this.VerticalOffset + this.ScrollOffset);
        }

        public void LineLeft()
        {
            throw new NotImplementedException();
        }

        public void LineRight()
        {
            throw new NotImplementedException();
        }

        public void LineUp()
        {
            this.SetVerticalOffset(this.VerticalOffset - this.ScrollOffset);
        }

        public Rect MakeVisible(Visual visual, Rect rectangle)
        {
            return new Rect();
        }

        public void MouseWheelDown()
        {
            this.SetVerticalOffset(this.VerticalOffset + this.ScrollOffset);
        }

        public void MouseWheelLeft()
        {
            throw new NotImplementedException();
        }

        public void MouseWheelRight()
        {
            throw new NotImplementedException();
        }

        public void MouseWheelUp()
        {
            this.SetVerticalOffset(this.VerticalOffset - this.ScrollOffset);
        }

        public void PageDown()
        {
            this.SetVerticalOffset(this.VerticalOffset + this.viewPort.Height);
        }

        public void PageLeft()
        {
            throw new NotImplementedException();
        }

        public void PageRight()
        {
            throw new NotImplementedException();
        }

        public void PageUp()
        {
            this.SetVerticalOffset(this.VerticalOffset - this.viewPort.Height);
        }

        public void SetHorizontalOffset(double offset)
        {
            throw new NotImplementedException();
        }

        public void SetVerticalOffset(double offset)
        {
            if (offset < 0 || this.viewPort.Height >= this.extent.Height)
                offset = 0;
            else
                if (offset + this.viewPort.Height >= this.extent.Height)
                offset = this.extent.Height - this.viewPort.Height;

            this.offset.Y = offset;
            this.ScrollOwner?.InvalidateScrollInfo();
            this.trans.Y = -offset;
            this.InvalidateMeasure();
            //接下来会触发MeasureOverride()
        }
        #endregion
    }
}
```

```c++
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    HelloWorld.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
//
// This example...
//
#include<math.h>
#include <vtkSmartPointer.h>
#include <vtkVectorText.h>
#include <vtkLinearExtrusionFilter.h>
#include <vtkTriangleFilter.h>
#include "vtkFeatureEdges.h"
#include <vtkDataSetMapper.h>
#include "vtkPolyData.h"
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPLYReader.h>
#include <iostream>
#include "vtkPLYReader.h"
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include "vtkAppendPolyData.h"
#include "vtkCleanPolyData.h"
#include "vtkSelection.h"
#include "vtkFillHolesFilter.h"
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkArrowSource.h>
#include <vtkMaskPoints.h>
#include <vtkGlyph3D.h>
#include "vtkCutter.h"
#include "vtkPlane.h"
#include "vtkClipPolyData.h"
#include "vtkStripper.h"
#include "vtkCenterOfMass.h"
#include "vtkMath.h"
#include "vtkMergePoints.h"
#include "vtkClipClosedSurface.h"
#include "vtkPlaneCollection.h"
#include <opencv2/highgui/highgui.hpp>  
#include <opencv2/core/core.hpp> 
#include<opencv2/core/mat.hpp>
#include<opencv2/core/types_c.h>
#include<opencv2/core/core_c.h>

void cvFitPlane(const CvMat* points, float* plane);
double DistancePointToPlane(double point[], double center[], double vector[]);
int main(int, char*[])
{
  /*vtkRandomGraphSource* source = vtkRandomGraphSource::New();

  vtkGraphLayoutView* view = vtkGraphLayoutView::New();
  view->SetRepresentationFromInputConnection(
    source->GetOutputPort());

  view->ResetCamera();
  view->Render();
  view->GetInteractor()->Start();

  source->Delete();
  view->Delete();*/
  //创建绘制器对象
	// Create vector text
	// C:\\Users\\zhangxinyan\\Desktop\\testobj4.ply   G:\\mesh\\30m10-new-cut-combine.ply
	std::string filePath = "C:\\Users\\zhangxinyan\\Desktop\\testobj4.ply" ;
	vtkSmartPointer<vtkPLYReader> reader =
		vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(filePath.c_str());
	
	
	//获取边界
	vtkSmartPointer<vtkFeatureEdges> featureEdges =
		vtkSmartPointer<vtkFeatureEdges>::New();
	featureEdges->SetInputConnection(reader->GetOutputPort());
	featureEdges->BoundaryEdgesOn();
	featureEdges->FeatureEdgesOff();
	featureEdges->ManifoldEdgesOff();
	featureEdges->NonManifoldEdgesOff();
	featureEdges->Update();

	

	vtkSmartPointer<vtkCenterOfMass> centerOfMassFilter =
		vtkSmartPointer<vtkCenterOfMass>::New();
	centerOfMassFilter->SetInputData(reader->GetOutput());
	centerOfMassFilter->SetUseScalarsAsWeights(false);
	centerOfMassFilter->Update();
	double readerCenter[3];
	centerOfMassFilter->GetCenter(readerCenter);

	//获取边缘polydata
	vtkPolyData* edgeData = featureEdges->GetOutput();
	//点数
	cout <<"edge point number:"<< edgeData->GetNumberOfPoints() << endl;


	//获取边缘点集
	int edgePointCount = edgeData->GetNumberOfPoints();
	CvMat* points_mat = cvCreateMat(edgePointCount, 3, CV_32FC1);
	for (int i = 0; i < edgePointCount; i++)
	{
		double loc[3];
		edgeData->GetPoint(i,loc);
		points_mat->data.fl[i * 3 + 0] = loc[0];//x
		points_mat->data.fl[i * 3 + 1] = loc[1];//  Y的坐标值
		points_mat->data.fl[i * 3 + 2] = loc[2];//y
	}
	float plane12[4] = { 0 };//定义用来储存平面参数的数组 
	cvFitPlane(points_mat, plane12);//调用方程
	double plane12Spoint[] = { 1,2,0 };
	plane12Spoint[2]=(-plane12[3] - plane12[0] - plane12[1] * 2) / plane12[2];

	//投影点
	vtkSmartPointer<vtkPlane> projectionPlane = vtkSmartPointer<vtkPlane>::New();
	projectionPlane->SetOrigin(plane12Spoint);
	projectionPlane->SetNormal(plane12[0], plane12[1], plane12[2]);
	double projectionPoint[3] = {0,0,0};
	projectionPlane->ProjectPoint(readerCenter,projectionPoint);
	double extrudeVecter[3] = { 0,0,0 };
	extrudeVecter[0] = readerCenter[0] - projectionPoint[0];
	extrudeVecter[1] = readerCenter[1] - projectionPoint[1];
	extrudeVecter[2] = readerCenter[2] - projectionPoint[2];
	double twoPointsLength = sqrt((pow(extrudeVecter[0], 2) + pow(extrudeVecter[1], 2) + pow(extrudeVecter[2], 2)));
	double extrudeUnitVector[3] = { (1 / twoPointsLength) * extrudeVecter[0],(1 / twoPointsLength) * extrudeVecter[1],(1 / twoPointsLength) * extrudeVecter[2] };


	//normalVector=()
	// Apply linear extrusion 
	vtkSmartPointer<vtkLinearExtrusionFilter> extrude =
		vtkSmartPointer<vtkLinearExtrusionFilter>::New();
	extrude->SetInputConnection(featureEdges->GetOutputPort());

	extrude->SetExtrusionTypeToNormalExtrusion();
	extrude->SetVector(-extrudeUnitVector[0]*60, -extrudeUnitVector[1]*60, -extrudeUnitVector[2] * 60);
	
	extrude->Update();

	//合并点
	vtkSmartPointer<vtkAppendPolyData> appendExtrude = vtkSmartPointer<vtkAppendPolyData>::New();
	appendExtrude->AddInputData(reader->GetOutput());
	appendExtrude->AddInputData(extrude->GetOutput());
	appendExtrude->Update();
	vtkSmartPointer<vtkMergePoints> mergeExtrude = vtkSmartPointer<vtkMergePoints>::New();
	mergeExtrude->InitPointInsertion(appendExtrude->GetOutput()->GetPoints(), appendExtrude->GetOutput()->GetBounds());
	mergeExtrude->SetDataSet(appendExtrude->GetOutput());
	mergeExtrude->BuildLocator();
	mergeExtrude->Update();
	//mergeExtrude->GetDataSet()
	
	

	//平移挤边
	/*vtkSmartPointer<vtkTransform> translate = vtkSmartPointer<vtkTransform>::New();
	translate->Translate(0, 0, 0);
	vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	transformFilter->SetInputConnection(extrude->GetOutputPort());
	transformFilter->SetTransform(translate);
	transformFilter->Update();*/
	//多边形转三角形
	
	////挤出边三角化
	//vtkSmartPointer<vtkTriangleFilter> triangleFilter =
	//	vtkSmartPointer<vtkTriangleFilter>::New();
	//triangleFilter->SetInputConnection(extrude->GetOutputPort());
	//////原模型三角化
	//vtkSmartPointer<vtkTriangleFilter> inputTriangleFilter =
	//	vtkSmartPointer<vtkTriangleFilter>::New();
	//inputTriangleFilter->SetInputConnection(reader->GetOutputPort());

																		
	////移除重复点
	//vtkSmartPointer<vtkCleanPolyData> cleanFilter =
	//	vtkSmartPointer<vtkCleanPolyData>::New();
	//cleanFilter->SetInputConnection(appendFilter->GetOutputPort());
	//cleanFilter->Update();
	//获取边缘边
	/*vtkSmartPointer<vtkFeatureEdges> edgeCombined = vtkSmartPointer<vtkFeatureEdges>::New();
	edgeCombined->SetInputConnection(appendFilter->GetOutputPort());
	edgeCombined->BoundaryEdgesOn();
	edgeCombined->FeatureEdgesOff();
	edgeCombined->ManifoldEdgesOff();
	edgeCombined->NonManifoldEdgesOff();
	edgeCombined->Update();*/


	//补洞
	/*vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter = vtkSmartPointer<vtkFillHolesFilter>::New();
	fillHolesFilter->SetInputData(reader->GetOutput());
	fillHolesFilter->SetHoleSize(100000.0);
	fillHolesFilter->Update();*/

	

	//
	double lowestPoint[3] = { 0,0,0 };
	edgeData->GetPoint(0, lowestPoint);
	double distence = DistancePointToPlane(lowestPoint, plane12Spoint, extrudeUnitVector);
	for (int i = 1; i < edgeData->GetNumberOfPieces(); i++)
	{
		
		double pointI[3] = { 0,0,0 };
		edgeData->GetPoint(i, pointI);
		if (DistancePointToPlane(pointI, plane12Spoint, extrudeUnitVector)<distence)
		{
			lowestPoint[0] = pointI[0];
			lowestPoint[1] = pointI[1];
			lowestPoint[2] = pointI[2];
		}
	}
	//切平面

	vtkSmartPointer<vtkPlane> cPlane = vtkSmartPointer<vtkPlane>::New();
	
	//设置裁剪中心
	cPlane->SetOrigin(extrude->GetOutput()->GetCenter());
	//cPlane->SetOrigin(lowestPoint);
	//cPlane->SetNormal(plane12[0], plane12[1], plane12[2]);
	cPlane->SetNormal(extrudeUnitVector);

	vtkSmartPointer<vtkPlaneCollection> planes =
		vtkSmartPointer<vtkPlaneCollection>::New();
	planes->AddItem(cPlane);
	//设置裁切 1
	vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
	clipper->SetInputData(extrude->GetOutput());
	clipper->SetClipFunction(cPlane);
	clipper->SetValue(0);
	clipper->Update();
	
	vtkSmartPointer<vtkClipClosedSurface> clipper2 =
		vtkSmartPointer<vtkClipClosedSurface>::New();
	clipper2->SetInputData(extrude->GetOutput());
	clipper2->SetClippingPlanes(planes);
	clipper2->SetActivePlaneId(0);
	clipper2->Update();
	//设置裁剪2

	//切面补面


	// Now extract feature edges
	auto boundaryEdges = vtkSmartPointer<vtkFeatureEdges>::New();
	boundaryEdges->SetInputData(clipper->GetOutput());
	boundaryEdges->BoundaryEdgesOn();
	boundaryEdges->FeatureEdgesOff();
	boundaryEdges->NonManifoldEdgesOff();
	boundaryEdges->ManifoldEdgesOff();

	auto boundaryStrips = vtkSmartPointer<vtkStripper>::New();
	boundaryStrips->SetInputConnection(boundaryEdges->GetOutputPort());
	boundaryStrips->Update();

	auto boundaryPoly = vtkSmartPointer<vtkPolyData>::New();
	boundaryPoly->SetPoints(boundaryStrips->GetOutput()->GetPoints());
	boundaryPoly->SetPolys(boundaryStrips->GetOutput()->GetLines());
	//三角化
	auto cutTriangles = vtkSmartPointer<vtkTriangleFilter>::New();
	cutTriangles->SetInputData(boundaryPoly);
	cutTriangles->Update();

	//模型合并
	vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
	appendFilter->AddInputData(reader->GetOutput());
	//appendFilter->AddInputData(transformFilter->GetOutput());
	appendFilter->AddInputData(clipper2->GetOutput());
	appendFilter->AddInputData(cutTriangles->GetOutput());
	appendFilter->Update();
	

	auto appendEdge = vtkSmartPointer<vtkFeatureEdges>::New();
	appendEdge->AddInputConnection(appendFilter->GetOutputPort());
	appendEdge->BoundaryEdgesOn();
	appendEdge->FeatureEdgesOff();
	appendEdge->NonManifoldEdgesOff();
	appendEdge->ManifoldEdgesOff();
	//查看模型法线方向
	vtkSmartPointer<vtkPolyDataNormals> modelNormal = vtkSmartPointer<vtkPolyDataNormals>::New();
	modelNormal->SetInputConnection(appendFilter->GetOutputPort());
	modelNormal->SetComputeCellNormals(0);
	modelNormal->SetComputePointNormals(1);
	modelNormal->SetAutoOrientNormals(1);
	modelNormal->SetSplitting(0);
	modelNormal->Update();

	vtkSmartPointer<vtkArrowSource> arrow = vtkSmartPointer<vtkArrowSource>::New();
	arrow->Update();

	vtkSmartPointer<vtkMaskPoints> mask = vtkSmartPointer<vtkMaskPoints>::New();
	mask->SetInputConnection(modelNormal->GetOutputPort());
	mask->SetMaximumNumberOfPoints(300);
	mask->RandomModeOn();
	mask->Update();

	vtkSmartPointer<vtkGlyph3D> glyph = vtkSmartPointer<vtkGlyph3D>::New();
	glyph->SetInputData(mask->GetOutput());
	glyph->SetSourceData(arrow->GetOutput());//每一点用箭头代替
	glyph->SetVectorModeToUseNormal();//设置向量显示模式和法向量一致
	glyph->SetScaleFactor(5); //设置伸缩比例
	glyph->Update();
	/////////////////

	vtkSmartPointer<vtkDataSetMapper> mapper =
		vtkSmartPointer<vtkDataSetMapper>::New();
	mapper->SetInputConnection(clipper2->GetOutputPort());

	vtkSmartPointer<vtkDataSetMapper> mapperGlyph =
		vtkSmartPointer<vtkDataSetMapper>::New();
	mapperGlyph->SetInputConnection(glyph->GetOutputPort());

	/*vtkSmartPointer<vtkDataSetMapper> inputMapper =
		vtkSmartPointer<vtkDataSetMapper>::New();
	mapper->SetInputConnection(inputTriangleFilter->GetOutputPort());*/

	//渲染场景
	vtkSmartPointer<vtkActor> actor =
		vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	//actor->GetProperty()->SetRepresentationToWireframe();
	actor->GetProperty()->SetColor(0.8900, 0.8100, 0.3400);

	vtkSmartPointer<vtkActor> glyphActor =
		vtkSmartPointer<vtkActor>::New();
	glyphActor->SetMapper(mapperGlyph);
	glyphActor->GetProperty()->SetColor(1, 0, 0);

	/*vtkSmartPointer<vtkActor> actor1 =
		vtkSmartPointer<vtkActor>::New();
	actor1->SetMapper(inputMapper);
	actor1->GetProperty()->SetColor(0.8900, 0.8100, 0.3400);*/

	//渲染窗口
	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();

	vtkSmartPointer<vtkRenderer> renderer =
		vtkSmartPointer<vtkRenderer>::New();
	renderer->SetBackground(.4, .5, .7);
	renderer->AddActor(actor);
	//renderer->AddActor(glyphActor);
	renderer->ResetCamera();
	// Generate an interesting view
	renderer->ResetCamera();
	renderer->GetActiveCamera()->Azimuth(30);
	renderer->GetActiveCamera()->Elevation(30);
	renderer->GetActiveCamera()->Dolly(1.0);
	renderer->ResetCameraClippingRange();
	

	renderWindow->AddRenderer(renderer);

	
	//renderer->AddActor(actor1);

	

	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	renderWindow->Render();
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
  return 0;
}




void cvFitPlane(const CvMat* points, float* plane) {
	// Estimate geometric centroid.
	int nrows = points->rows;
	int ncols = points->cols;
	int type = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for (int c = 0; c < ncols; c++) {
		for (int r = 0; r < nrows; r++)
		{
			centroid->data.fl[c] += points->data.fl[ncols * r + c];
		}
		centroid->data.fl[c] /= nrows;
	}
	// Subtract geometric centroid from each point.
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for (int r = 0; r < nrows; r++)
		for (int c = 0; c < ncols; c++)
			points2->data.fl[ncols * r + c] = points->data.fl[ncols * r + c] - centroid->data.fl[c];
	// Evaluate SVD of covariance matrix.
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);
	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
	cvSVD(A, W, NULL, V, CV_SVD_V_T);
	// Assign plane coefficients by singular vector corresponding to smallest singular value.
	plane[ncols] = 0;
	for (int c = 0; c < ncols; c++) {
		plane[c] = V->data.fl[ncols * (ncols - 1) + c];
		plane[ncols] += plane[c] * centroid->data.fl[c];
	}
	// Release allocated resources.
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}

double DistancePointToPlane(double point[], double center[], double vector[])
{
	vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
	plane->SetOrigin(center);
	plane->SetNormal(vector);
	double ProjectPoint[3] = { 0,0,0 };
	plane->ProjectPoint(point, ProjectPoint);
	return vtkMath::Distance2BetweenPoints(point, ProjectPoint);
}
```



```c++ 
	for (int i = 0; i <= pointsNum; i++)
	{
		if (i!=pointsNum)
		{
			points->InsertPoint(i, edgeData->GetPoint(i));
		}
		else
		{
			points->InsertPoint(i, new double[3]{ 0,0,0 });
		}
		scalars->InsertTuple1(i, i);
	}
	for (size_t i = 0; i < points->GetNumberOfPoints(); i++)
	{
		auto loc = points->GetPoint(i);
		cout << "x:" << loc[0] << "  y:" << loc[1] << "  z:" << loc[2] << endl;
	}
	for (int i = 0; i < pointsNum; i++)
	{

		if (i+1<pointsNum)
		{
			array<vtkIdType, 3> cellData = { {pointsNum ,i,i + 1 } };
			polys->InsertNextCell( vtkIdType(3), cellData.data());
		}
		else
		{
			array<vtkIdType, 3> cellData = { {pointsNum ,i,0} };
			polys->InsertNextCell(vtkIdType(3),cellData.data());
		}
	}
```




