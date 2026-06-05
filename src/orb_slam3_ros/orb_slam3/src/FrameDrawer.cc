/** 
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

// ★★★ 修复版本：添加了所有必要的边界检查，防止双目模式下崩溃 ★★★

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM3
{

FrameDrawer::FrameDrawer(Atlas* pAtlas):both(false),mpAtlas(pAtlas)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mImRight = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys;
    vector<int> vMatches;
    vector<cv::KeyPoint> vCurrentKeys;
    vector<bool> vbVO, vbMap;
    vector<pair<cv::Point2f, cv::Point2f> > vTracks;
    int state;
    vector<float> vCurrentDepth;
    float thDepth;

    Frame currentFrame;
    vector<bool> vInfoSelected;
    vector<bool> vInfoCandidate;
    vector<unsigned char> vInfoCandType;

    vector<MapPoint*> vpLocalMap;
    vector<cv::KeyPoint> vMatchesKeys;
    vector<MapPoint*> vpMatchedMPs;
    vector<cv::KeyPoint> vOutlierKeys;
    vector<MapPoint*> vpOutlierMPs;
    map<long unsigned int, cv::Point2f> mProjectPoints;
    map<long unsigned int, cv::Point2f> mMatchedInImage;

    cv::Scalar standardColor(255,255,0);
    cv::Scalar odometryColor(255,255,0);

    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
            vTracks = mvTracks;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            currentFrame = mCurrentFrame;
            vInfoSelected = currentFrame.mvInfoSelected;
            vInfoCandidate = currentFrame.mvInfoCandidate;
            vInfoCandType  = currentFrame.mvInfoCandType;

            vpLocalMap = mvpLocalMap;
            vMatchesKeys = mvMatchedKeys;
            vpMatchedMPs = mvpMatchedMPs;
            vOutlierKeys = mvOutlierKeys;
            vpOutlierMPs = mvpOutlierMPs;
            mProjectPoints = mmProjectPoints;
            mMatchedInImage = mmMatchedInImage;

            vCurrentDepth = mvCurrentDepth;
            thDepth = mThDepth;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    }

    // ★ 边界检查：确保图像有效
    if(im.empty())
    {
        im = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        if(imWidth > 0 && imHeight > 0)
            cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3)
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0 && vMatches[i] < (int)vCurrentKeys.size() && i < vIniKeys.size())
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,standardColor);
            }
        }
        for(vector<pair<cv::Point2f, cv::Point2f> >::iterator it=vTracks.begin(); it!=vTracks.end(); it++)
        {
            cv::Point2f pt1,pt2;
            if(imageScale != 1.f)
            {
                pt1 = (*it).first / imageScale;
                pt2 = (*it).second / imageScale;
            }
            else
            {
                pt1 = (*it).first;
                pt2 = (*it).second;
            }
            cv::line(im,pt1,pt2, standardColor,5);
        }
    }
    else if(state==Tracking::OK)
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            // ★★★ 关键修复：边界检查 ★★★
            if(i >= (int)vbVO.size() || i >= (int)vbMap.size())
                continue;

            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = vCurrentKeys[i].pt / imageScale;
                    float px = vCurrentKeys[i].pt.x / imageScale;
                    float py = vCurrentKeys[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = vCurrentKeys[i].pt;
                    pt1.x=vCurrentKeys[i].pt.x-r;
                    pt1.y=vCurrentKeys[i].pt.y-r;
                    pt2.x=vCurrentKeys[i].pt.x+r;
                    pt2.y=vCurrentKeys[i].pt.y+r;
                }

                if(vbMap[i])
                {
                    cv::circle(im,point,3,standardColor,1);
                    mnTracked++;
                }
                else
                {
                    cv::circle(im,point,3,odometryColor,1);
                    mnTrackedVO++;
                }
            }
        }

        if (!vInfoCandidate.empty())
        {
            int nDraw = std::min((int)vInfoCandidate.size(), (int)vCurrentKeys.size());
            cv::Scalar candColor(255,255,0);

            for (int i = 0; i < nDraw; ++i)
            {
                if (!vInfoCandidate[i]) continue;

                cv::Point2f pt = (imageScale != 1.f) ? (vCurrentKeys[i].pt / imageScale)
                                                    :  vCurrentKeys[i].pt;
                cv::circle(im, pt, 3, candColor, 1);
            }
        }

        if (!vInfoSelected.empty())
        {
            cv::Scalar infoColor(255,0,255);
            int nInfo = std::min(n, (int)vInfoSelected.size());

            for (int i = 0; i < nInfo; ++i)
            {
                if (!vInfoSelected[i]) continue;
                if (i >= (int)vbMap.size() || !vbMap[i]) continue;

                cv::Point2f pt;
                if (imageScale != 1.f)
                    pt = vCurrentKeys[i].pt / imageScale;
                else
                    pt = vCurrentKeys[i].pt;

                cv::circle(im, pt, 4, infoColor, 2);
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}

cv::Mat FrameDrawer::DrawRightFrame(float imageScale)
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys;
    vector<int> vMatches;
    vector<cv::KeyPoint> vCurrentKeys;
    vector<bool> vbVO, vbMap;
    int state;
    Frame currentFrame;
    vector<bool> vInfoSelected;
    vector<bool> vInfoCandidate;
    vector<unsigned char> vInfoCandType;

    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mImRight.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeysRight;
            vbVO = mvbVO;
            vbMap = mvbMap;
            currentFrame = mCurrentFrame;
            vInfoSelected = currentFrame.mvInfoSelected;
            vInfoCandidate = currentFrame.mvInfoCandidate;
            vInfoCandType  = currentFrame.mvInfoCandType;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeysRight;
        }
    }

    // ★ 边界检查：确保图像有效
    if(im.empty())
    {
        im = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0,0,0));
    }

    if(imageScale != 1.f)
    {
        int imWidth = im.cols / imageScale;
        int imHeight = im.rows / imageScale;
        if(imWidth > 0 && imHeight > 0)
            cv::resize(im, im, cv::Size(imWidth, imHeight));
    }

    if(im.channels()<3)
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    if(state==Tracking::NOT_INITIALIZED)
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0 && vMatches[i] < (int)vCurrentKeys.size() && i < vIniKeys.size())
            {
                cv::Point2f pt1,pt2;
                if(imageScale != 1.f)
                {
                    pt1 = vIniKeys[i].pt / imageScale;
                    pt2 = vCurrentKeys[vMatches[i]].pt / imageScale;
                }
                else
                {
                    pt1 = vIniKeys[i].pt;
                    pt2 = vCurrentKeys[vMatches[i]].pt;
                }
                cv::line(im,pt1,pt2,cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK)
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = mvCurrentKeysRight.size();
        const int Nleft = mvCurrentKeys.size();

        for(int i=0;i<n;i++)
        {
            // ★★★ 关键修复：双目右目边界检查 ★★★
            int idx = i + Nleft;
            if(idx >= (int)vbVO.size() || idx >= (int)vbMap.size())
                continue;

            if(vbVO[idx] || vbMap[idx])
            {
                cv::Point2f pt1,pt2;
                cv::Point2f point;
                if(imageScale != 1.f)
                {
                    point = mvCurrentKeysRight[i].pt / imageScale;
                    float px = mvCurrentKeysRight[i].pt.x / imageScale;
                    float py = mvCurrentKeysRight[i].pt.y / imageScale;
                    pt1.x=px-r;
                    pt1.y=py-r;
                    pt2.x=px+r;
                    pt2.y=py+r;
                }
                else
                {
                    point = mvCurrentKeysRight[i].pt;
                    pt1.x=mvCurrentKeysRight[i].pt.x-r;
                    pt1.y=mvCurrentKeysRight[i].pt.y-r;
                    pt2.x=mvCurrentKeysRight[i].pt.x+r;
                    pt2.y=mvCurrentKeysRight[i].pt.y+r;
                }

                if(vbMap[idx])
                {
                    cv::circle(im,point,3,cv::Scalar(255,255,0),1);
                    mnTracked++;
                }
                else
                {
                    cv::circle(im,point,3,cv::Scalar(255,255,0),1);
                    mnTrackedVO++;
                }
            }
        }

        if(!vInfoCandidate.empty())
        {
            cv::Scalar candColor(255,255,0);

            for(int i = 0; i < n; ++i)
            {
                const int idxFrame = Nleft + i;

                if(idxFrame < 0 || idxFrame >= (int)vInfoCandidate.size())
                    continue;
                if(!vInfoCandidate[idxFrame])
                    continue;

                cv::Point2f pt;
                if(imageScale != 1.f)
                    pt = mvCurrentKeysRight[i].pt / imageScale;
                else
                    pt = mvCurrentKeysRight[i].pt;

                cv::circle(im, pt, 3, candColor, 1);
            }
        }

        if(!vInfoSelected.empty())
        {
            cv::Scalar infoColor(255,0,255);

            for(int i=0; i<n; ++i)
            {
                const int idxFrame = Nleft + i;
                if(idxFrame < 0 || idxFrame >= (int)vInfoSelected.size())
                    continue;
                if(!vInfoSelected[idxFrame])
                    continue;
                if(idxFrame >= (int)vbMap.size() || !vbMap[idxFrame])
                    continue;

                cv::Point2f pt;
                if(imageScale != 1.f)
                    pt = mvCurrentKeysRight[i].pt / imageScale;
                else
                    pt = mvCurrentKeysRight[i].pt;

                cv::circle(im, pt, 4, infoColor, 2);
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}



void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    // ★★★ 关键修复：防止空图像崩溃 ★★★
    if(im.empty() || im.rows <= 0 || im.cols <= 0)
    {
        imText = cv::Mat(50, 640, CV_8UC3, cv::Scalar(0,0,0));
        cv::putText(imText, "Waiting for image...", cv::Point(10, 30),
                    cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255,255,255), 1, 8);
        return;
    }

    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        s << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);
}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mThDepth = pTracker->mCurrentFrame.mThDepth;
    mvCurrentDepth = pTracker->mCurrentFrame.mvDepth;

    if(both){
        mvCurrentKeysRight = pTracker->mCurrentFrame.mvKeysRight;
        pTracker->mImRight.copyTo(mImRight);
        N = mvCurrentKeys.size() + mvCurrentKeysRight.size();
    }
    else{
        N = mvCurrentKeys.size();
    }

    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    mCurrentFrame = pTracker->mCurrentFrame;
    mmProjectPoints = mCurrentFrame.mmProjectPoints;
    mmMatchedInImage.clear();

    mvpLocalMap = pTracker->GetLocalMapMPS();
    mvMatchedKeys.clear();
    mvMatchedKeys.reserve(N);
    mvpMatchedMPs.clear();
    mvpMatchedMPs.reserve(N);
    mvOutlierKeys.clear();
    mvOutlierKeys.reserve(N);
    mvpOutlierMPs.clear();
    mvpOutlierMPs.reserve(N);

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        // ★ 边界检查
        int nMapPoints = pTracker->mCurrentFrame.mvpMapPoints.size();
        int nOutliers = pTracker->mCurrentFrame.mvbOutlier.size();

        for(int i=0;i<N;i++)
        {
            // ★ 边界检查：确保不越界访问
            if(i >= nMapPoints || i >= nOutliers)
                continue;

            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;

                    // ★ 边界检查
                    if(i < (int)mvCurrentKeys.size())
                        mmMatchedInImage[pMP->mnId] = mvCurrentKeys[i].pt;
                }
                else
                {
                    mvpOutlierMPs.push_back(pMP);
                    if(i < (int)mvCurrentKeys.size())
                        mvOutlierKeys.push_back(mvCurrentKeys[i]);
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM3