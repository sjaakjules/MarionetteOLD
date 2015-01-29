using System;
using System.Drawing;
using Grasshopper.Kernel;
using Microsoft.Xna.Framework;

namespace Marionette
{
    public class MarionetteInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "Marionette";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("d492c705-79c3-4fee-bc25-e1e1b063fa45");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
